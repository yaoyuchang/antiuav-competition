"""L1 space partitioning: altitude bands + lateral corridors on a planned
global-guide route.

Used both for the first leg (formal_case in run_phase6b_final_validation.py)
and the post-3500m-switch replan (FinalValidationEvaluator.run()), which is
why this lives in mamp/ rather than in the top-level script -- the evaluator
cannot import from a top-level script that itself imports the evaluator.
"""
import numpy as np

from ..configs import subject3_config as config


def _interpolate_at_x(waypoints, x_target):
    """Return the point on the polyline at ``x_target``, or None if unreachable."""
    for index in range(len(waypoints) - 1):
        lower, upper = waypoints[index], waypoints[index + 1]
        if upper[0] > lower[0] and lower[0] <= x_target <= upper[0]:
            ratio = (x_target - lower[0]) / (upper[0] - lower[0])
            return lower + ratio * (upper - lower)
    return None


def band_waypoints(guide_planner, waypoints, band_altitude, lateral_offset=0.,
                   climb_fraction=None, cruise_end_fraction=None,
                   descent_end_fraction=None):
    """Lift the cruise section of a planned guide to ``band_altitude`` and push
    it sideways by ``lateral_offset``.

    Raising the route is safe by construction -- obstacles are ground-anchored
    prisms, so a laterally clear path stays clear as it climbs -- but shifting
    it sideways is not, and neither are the climb and descent legs.  Every
    resulting segment is therefore re-checked against the real obstacle
    geometry rather than against the A* occupancy grid: the planner's own LOS
    pruning works off that same geometry and would otherwise straighten the
    climb away.

    The three breakpoints are fractions of THIS route's own (start_x, goal_x)
    span, not absolute metres -- so the same function serves both the first
    leg (0->5000 m) and a much shorter, arbitrarily-positioned post-switch leg.
    Both climb/descent and the lateral offset share one ramp factor (0 at the
    start, 1 across the cruise, 0 again by the descent breakpoint) so they wind
    in and out together. Past that point the returned route is identical to
    the planned one, leaving whatever comes after (terminal phase, or -- for a
    first-leg route -- the goal switch) exactly as it was.

    Returns None when the route cannot be shaped (non-monotonic in x, or a
    moved segment clips an obstacle); the caller then falls back.
    """
    start_x, start_altitude = float(waypoints[0][0]), float(waypoints[0][1])
    goal_x = float(waypoints[-1][0])
    span = goal_x - start_x
    climb_end = start_x + (config.GUIDE_BAND_CLIMB_FRACTION if climb_fraction is None
                           else climb_fraction) * span
    cruise_end = start_x + (config.GUIDE_BAND_CRUISE_END_FRACTION if cruise_end_fraction is None
                            else cruise_end_fraction) * span
    descent_end = start_x + (config.GUIDE_BAND_DESCENT_END_FRACTION if descent_end_fraction is None
                             else descent_end_fraction) * span
    if not start_x < climb_end < cruise_end < descent_end < goal_x:
        return None
    rejoin = _interpolate_at_x(waypoints, descent_end)
    if rejoin is None:
        return None
    rejoin_altitude = float(rejoin[1])

    def ramp_at(x):
        if x <= climb_end:
            return (x - start_x) / (climb_end - start_x)
        if x <= cruise_end:
            return 1.
        if x >= descent_end:
            return 0.
        return (descent_end - x) / (descent_end - cruise_end)

    def altitude_at(x, planned_altitude):
        if x <= climb_end:
            ratio = (x - start_x) / (climb_end - start_x)
            return start_altitude + ratio * (band_altitude - start_altitude)
        if x <= cruise_end:
            return band_altitude
        if x >= descent_end:
            return planned_altitude
        ratio = (x - cruise_end) / (descent_end - cruise_end)
        return band_altitude + ratio * (rejoin_altitude - band_altitude)

    x_values = sorted(set([float(point[0]) for point in waypoints] +
                          [climb_end, cruise_end, descent_end]))
    banded = []
    for x in x_values:
        point = _interpolate_at_x(waypoints, x)
        if point is None:
            return None
        banded.append([point[0], altitude_at(point[0], float(point[1])),
                       point[2] + ramp_at(point[0]) * lateral_offset])
    banded = np.asarray(banded, dtype='float64')
    keep = np.r_[True, np.linalg.norm(np.diff(banded, axis=0), axis=1) > 1e-10]
    banded = banded[keep]
    if len(banded) < 2:
        return None
    for index in range(len(banded) - 1):
        if not guide_planner.segment_is_static_safe(banded[index],
                                                    banded[index + 1]):
            return None
    return banded


def shaped_initial_route(guide_planner, waypoints, uav_index, goal_z,
                         lane_offset=None):
    """Return the banded/corridored first-leg route for one UAV, degrading on
    failure.

    ``lane_offset`` (from lane_assignment.assign_lane_offsets(), computed
    once for the whole fleet from where UAVs are actually headed after the
    goal switch, not from this UAV's own first-leg goal) is tried at full
    strength, then discounted back toward 0 in GUIDE_LANE_DISCOUNT_LADDER
    steps, then altitude-only, then the planned route untouched -- each step
    re-validated against real obstacle geometry inside band_waypoints().
    Discounting is needed for the same reason the old fixed corridor scales
    needed a ladder: shifting sideways can clip an obstacle even when the
    lane itself was checked reachable along a coarse probe grid.

    When GUIDE_LANE_ASSIGNMENT_ENABLED is False (or lane_offset is None,
    e.g. a caller that hasn't computed lane assignment), this falls back to
    the original fixed-scale corridor ladder -- goal_z*(scale-1), widest
    first -- which is what produced the last known-good baseline (583
    cycles) before lane assignment existed. Both goal_z and this fallback
    stay here, unremoved, specifically so that flipping the switch reproduces
    that exact prior behaviour instead of a re-derived approximation of it.
    """
    bands = config.GUIDE_ALTITUDE_BANDS
    if not bands:
        return waypoints
    band_altitude = bands[uav_index % len(bands)]
    if config.GUIDE_LANE_ASSIGNMENT_ENABLED and lane_offset is not None:
        for discount in config.GUIDE_LANE_DISCOUNT_LADDER:
            shaped = band_waypoints(guide_planner, waypoints, band_altitude,
                                    lane_offset * discount)
            if shaped is not None:
                return shaped
    else:
        # goal_z is monotonic in UAV index, so scaling it keeps corridor
        # order and goal order identical and the routes never cross as they
        # converge. Safe here because the first leg's goal_z stays within
        # +-130 m, well inside the +-680 m grid bound even at 2.5x.
        for scale in config.GUIDE_CORRIDOR_SCALES:
            shaped = band_waypoints(guide_planner, waypoints, band_altitude,
                                    goal_z * (scale - 1.))
            if shaped is not None:
                return shaped
    shaped = band_waypoints(guide_planner, waypoints, band_altitude)
    return waypoints if shaped is None else shaped


def shaped_switch_route(guide_planner, waypoints, current_altitude, goal_z):
    """Return the altitude-held post-switch replan route for one UAV.

    Unlike the first leg, this route starts wherever the UAV happens to be
    when it crosses GOAL_SWITCH_SLANT_RANGE, so the profile keeps the UAV at
    its CURRENT altitude (no climb -- it's already up there) instead of
    looking up a fixed band, and descends over the last fraction of this much
    shorter leg (see GUIDE_SWITCH_BAND_* in subject3_config.py).

    No lateral corridor here, deliberately: the three changed-goal groups
    already sit hundreds of metres apart in z (roughly 0 / -560 / +560), so
    they don't need artificial widening the way the tightly-packed first-leg
    goals (+-130 m) did -- and unlike there, entry z and goal_z can already be
    far apart at the switch (a UAV can be at z=-80 heading to a goal at
    z=-560), so scaling that gap by even 1.5x pushes the route hundreds of
    metres further out and off the +-680 m grid, which is exactly what a first
    version of this function did and it failed real UAVs outright. What
    actually causes the switch-region pileup is UAVs from different altitude
    bands all flattening back to a common altitude at nearly the same moment;
    holding each UAV's own altitude longer addresses that directly, so a
    lateral corridor isn't needed on top of it.

    ``goal_z`` is accepted for interface symmetry with shaped_initial_route
    and to leave room for a future corridor scheme, but is currently unused.

    Always returns a usable route (falls back to the plain planned waypoints
    if shaping fails) -- the caller's own guide.success check on the
    underlying plan() call remains the only hard failure point, so adding
    this shaping can never make the switch replan MORE likely to fail the
    mission than it already was.
    """
    del goal_z
    shaped = band_waypoints(
        guide_planner, waypoints, current_altitude,
        climb_fraction=config.GUIDE_SWITCH_BAND_CLIMB_FRACTION,
        cruise_end_fraction=config.GUIDE_SWITCH_BAND_CRUISE_END_FRACTION,
        descent_end_fraction=config.GUIDE_SWITCH_BAND_DESCENT_END_FRACTION)
    return waypoints if shaped is None else shaped
