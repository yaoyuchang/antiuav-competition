"""Assign each UAV a lateral lane on the first leg, chosen by where it will
actually be heading after the goal switch -- not by its first-leg goal.

Why this exists (see the architecture doc / commit history for the full
measurement trail): the swarm's real congestion is not a place, it is an
EVENT -- the goal switch at GOAL_SWITCH_SLANT_RANGE, where every UAV abandons
its tightly-packed first-leg goal and beelines for a changed goal that can be
hundreds of metres away. Two purely-geometric congestion predictors (density
and lane-convergence, both tried and rejected) failed to find this because
the first-leg routes are uniformly dense end to end; the pileup only exists
because the routes are about to diverge, and divergence is a property of the
*second* leg, invisible to first-leg-only geometry. So instead of predicting
a hotspot, this module removes the event: every UAV flies the first leg
already offset toward the lane its POST-SWITCH destination lives in, so the
switch becomes a small correction instead of a sudden fan-out.

Grouping is by effective goal z, not by UAV index or first-leg goal sign --
both of those were tried and rejected. Index-based bypass assignment (an
earlier "north/south" idea) would have needed to know the destination up
front anyway; and first-leg-goal-sign grouping measurably fails here: UAVs
11-16 have first-leg goals at z in [0, 50] (nominally "north") but changed
goals at z in [-565, -540] (actually "south") -- 6/24 UAVs would cross lanes
exactly at the switch point, which is the one place that can least afford it.
"""
import numpy as np

from ..configs import subject3_config as config
from .route_shaping import _interpolate_at_x


def free_lanes(guide_planner, x, altitude, z_range=(-680., 680.), step=2.,
              min_width=8.):
    """Free lateral intervals at one (x, altitude) slice, from real geometry.

    General-purpose probe, also handy for read-only diagnostics: this is the
    same query used throughout the measurement trail above.
    """
    z_values = np.arange(z_range[0], z_range[1] + step, step)
    points = np.column_stack([np.full_like(z_values, x),
                              np.full_like(z_values, altitude), z_values])
    mask = guide_planner.points_are_static_safe(points)
    lanes, run = [], None
    for ok, z in zip(mask, z_values):
        if ok and run is None:
            run = [z, z]
        elif ok:
            run[1] = z
        elif run is not None:
            lanes.append((float(run[0]), float(run[1])))
            run = None
    if run is not None:
        lanes.append((float(run[0]), float(run[1])))
    return [lane for lane in lanes if lane[1] - lane[0] >= min_width]


def group_by_effective_goal(effective_goal_z):
    """Cluster UAV indices into destination groups by effective goal z.

    Splits wherever the gap between consecutive (sorted) goals is
    noticeably larger than the typical within-group spacing, so the number
    of groups is discovered from the data rather than assumed (this
    competition's own changed-goal data happens to produce 3 groups, but
    nothing here hard-codes that). The typical spacing is the MEDIAN gap,
    which stays small even if a couple of genuinely huge inter-group gaps
    are present, unlike the mean.
    """
    effective_goal_z = np.asarray(effective_goal_z, dtype='float64')
    order = np.argsort(effective_goal_z)
    sorted_z = effective_goal_z[order]
    if len(sorted_z) <= 1:
        return [order.tolist()]
    diffs = np.diff(sorted_z)
    typical = float(np.median(diffs)) if np.any(diffs > 1e-9) else 0.
    threshold = max(typical * config.GUIDE_LANE_GROUP_GAP_FACTOR,
                    config.GUIDE_LANE_GROUP_MIN_GAP)
    groups, current = [], [int(order[0])]
    for i, gap in enumerate(diffs):
        if gap > threshold:
            groups.append(current)
            current = []
        current.append(int(order[i + 1]))
    groups.append(current)
    return groups


def _cruise_probe_xs(waypoints, step):
    """x samples across the plateau where a lane offset is fully applied
    (ramp_at == 1 in band_waypoints), capped at the goal-switch range --
    geometry past the switch point is moot because a UAV that actually
    switches replans there anyway, and scaling a huge cross-leg gap into
    that tail is exactly what blew a route off the +-680 m grid the first
    time a version of this kind of shaping was tried on the switch leg.
    """
    start_x = float(waypoints[0][0])
    goal_x = float(waypoints[-1][0])
    span = goal_x - start_x
    climb_end = start_x + config.GUIDE_BAND_CLIMB_FRACTION * span
    descent_end = start_x + config.GUIDE_BAND_DESCENT_END_FRACTION * span
    hi = min(descent_end, config.GOAL_SWITCH_SLANT_RANGE)
    if hi <= climb_end:
        return np.array([climb_end])
    return np.arange(climb_end, hi + 1e-6, step)


def _natural_cruise_z(waypoints):
    """The route's own (unshaped) z at the midpoint of its cruise plateau --
    the baseline that a lane offset gets added on top of. Not assumed to be
    0 or equal to the goal z: measured, all 24 planned routes actually
    thread a shared lateral gap near z=-10 regardless of their own start/goal
    z (see subject3_config.py), so using the route's real value here rather
    than approximating it is what lets the resulting offset land the UAV at
    the intended lane centre instead of some scale of its own goal.
    """
    start_x = float(waypoints[0][0])
    goal_x = float(waypoints[-1][0])
    span = goal_x - start_x
    climb_end = start_x + config.GUIDE_BAND_CLIMB_FRACTION * span
    cruise_end = start_x + config.GUIDE_BAND_CRUISE_END_FRACTION * span
    mid_x = 0.5 * (climb_end + cruise_end)
    point = _interpolate_at_x(waypoints, mid_x)
    return float(point[2]) if point is not None else float(waypoints[0][2])


def assign_lane_offsets(guide_planner, routes, band_of_uav, effective_goal_z):
    """Return a per-UAV lateral offset array for shaped_initial_route().

    ``band_of_uav[i]`` is the altitude band VALUE (not index) UAV i cruises
    at. Grouping is purely by destination and is altitude-agnostic, so a
    group can span several bands; reachability is then checked at every
    altitude actually present in the group before committing to it.

    Each group's candidate lane centre starts at the group's own effective
    goals (i.e. UAVs are individually offset onto their own eventual
    destination, preserving intra-group order and spacing exactly -- no
    within-group widening is invented). If that is not reachable along the
    probed cruise stretch at every altitude the group uses, the whole
    group's candidate is scaled toward the centreline (z=0) and retried;
    giving up after GUIDE_LANE_SHRINK_MAX_TRIES leaves that group at offset
    0, which is not a hard failure -- shaped_initial_route()'s own
    band-only / plain-route fallbacks still apply on top of this.
    """
    n = len(effective_goal_z)
    effective_goal_z = np.asarray(effective_goal_z, dtype='float64')
    band_of_uav = np.asarray(band_of_uav, dtype='float64')
    offsets = np.zeros(n, dtype='float64')
    if n == 0:
        return offsets

    for group in group_by_effective_goal(effective_goal_z):
        altitudes = sorted(set(float(band_of_uav[i]) for i in group))
        probe_xs = np.unique(np.concatenate(
            [_cruise_probe_xs(routes[i], config.GUIDE_LANE_PROBE_X_STEP)
             for i in group]))
        candidates = effective_goal_z[group]
        scale = 1.0
        reachable = False
        for _ in range(config.GUIDE_LANE_SHRINK_MAX_TRIES):
            trial_z = candidates * scale
            reachable = True
            for altitude in altitudes:
                points = np.array(
                    [[x, altitude, z] for x in probe_xs for z in trial_z],
                    dtype='float64')
                if len(points) and not bool(np.all(
                        guide_planner.points_are_static_safe(points))):
                    reachable = False
                    break
            if reachable:
                break
            scale *= config.GUIDE_LANE_SHRINK_FACTOR
        lane_center = trial_z if reachable else np.zeros_like(candidates)
        for local_index, uav in enumerate(group):
            natural_z = _natural_cruise_z(routes[uav])
            offsets[uav] = lane_center[local_index] - natural_z
    return offsets
