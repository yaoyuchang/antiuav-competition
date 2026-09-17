"""Phase-6B frozen-system final validation matrix."""

import time

import numpy as np

from mamp.agents.obstacle import Obstacle
from mamp.configs import subject3_config as config
from mamp.envs.subject3_environment import Subject3Environment
from mamp.planners.global_guide import GlobalGuidePlanner, GlobalGuideTracker
from mamp.planners.goal_manager import DynamicGoalModel, GoalManager
from mamp.planners.receding_horizon_planner import RecedingHorizonPlanner
from mamp.simulation.final_validation_evaluator import (
    FinalValidationEvaluator, report_dict, timing_summary)


def _interpolate_at_x(waypoints, x_target):
    """Return the point on the polyline at ``x_target``, or None if unreachable."""
    for index in range(len(waypoints) - 1):
        lower, upper = waypoints[index], waypoints[index + 1]
        if upper[0] > lower[0] and lower[0] <= x_target <= upper[0]:
            ratio = (x_target - lower[0]) / (upper[0] - lower[0])
            return lower + ratio * (upper - lower)
    return None


def band_waypoints(guide_planner, waypoints, band_altitude):
    """Lift the cruise section of a planned guide to ``band_altitude``.

    The horizontal route is reused as planned -- it already avoids every static
    obstacle, and raising a laterally clear path keeps it clear, since obstacles
    are ground-anchored prisms.  Only the climb and descent legs are new
    geometry, so every resulting segment is re-checked against the real
    obstacles rather than against the A* occupancy grid: the planner's own
    pruning works off that same geometry and would otherwise straighten the
    climb away.

    Past ``GUIDE_BAND_DESCENT_END_X`` the returned route is identical to the
    planned one, so the terminal phase is left exactly as it was.

    Returns None when the route cannot be banded (non-monotonic in x, or a
    lifted segment clips an obstacle); the caller then keeps the flat guide.
    """
    climb_end = config.GUIDE_BAND_CLIMB_END_X
    cruise_end = config.GUIDE_BAND_CRUISE_END_X
    descent_end = config.GUIDE_BAND_DESCENT_END_X
    start_x, start_altitude = float(waypoints[0][0]), float(waypoints[0][1])
    if not start_x < climb_end < cruise_end < descent_end < float(waypoints[-1][0]):
        return None
    rejoin = _interpolate_at_x(waypoints, descent_end)
    if rejoin is None:
        return None
    rejoin_altitude = float(rejoin[1])

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
                       point[2]])
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


def formal_case(mode='fixed', seed=0, starts_csv=None, initial_goals_csv=None,
                changed_goals_csv=None):
    """Create the formal validation case, optionally from CSV configuration.

    Args:
        mode: Running mode ('fixed', 'switch', 'sin', 'random')
        seed: Random seed
        starts_csv: Path to start_points.csv (optional; default hardcoded case)
        initial_goals_csv: Path to goal_points_initial.csv (optional)
        changed_goals_csv: Path to goal_points_changed.csv (optional)

    Returns:
        (evaluator, starts, initial_goals).  ``evaluator`` additionally
        carries a ``single_planning_duration_seconds`` attribute: the
        wall-clock time to produce all 24 complete global guide paths in one
        shot, which is the official "single planning duration" metric
        definition confirmed with the organizers, distinct from the
        per-cycle rolling-horizon latency reported elsewhere.
    """
    environment = Subject3Environment(
        starts_csv=starts_csv,
        initial_goals_csv=initial_goals_csv,
        changed_goals_csv=changed_goals_csv
    )
    boundary = np.vstack((environment.starts, environment.initial_goals,
                          environment.changed_goals))
    guide_planner = GlobalGuidePlanner(environment.obstacles, boundary,
                                       vertical_reserve=10.)
    planning_start = time.perf_counter()
    bands = config.GUIDE_ALTITUDE_BANDS
    routes = []
    for uav, (start, goal) in enumerate(zip(environment.starts,
                                            environment.initial_goals)):
        guide = guide_planner.plan(start, goal)
        if not guide.success:
            raise RuntimeError('global guide failed for UAV {}: {}'.format(
                uav, guide.reason))
        banded = (None if not bands else
                  band_waypoints(guide_planner, guide.waypoints,
                                 bands[uav % len(bands)]))
        routes.append(guide.waypoints if banded is None else banded)
    single_planning_duration_seconds = time.perf_counter() - planning_start
    planners = [RecedingHorizonPlanner(
        GlobalGuideTracker(waypoints, turn_aware_enabled=True,
                           turn_accel_ref_ratio=.3), environment.obstacles,
        # Dense terminal points are only 5 m apart.  A zero capture speed lets
        # each UAV settle into its 3 m arrival ball without flying through an
        # adjacent UAV that is already holding position.
        goal=goal, terminal_capture_speed=0.)
        for waypoints, goal in zip(routes, environment.initial_goals)]
    if mode == 'fixed':
        managers = None
    else:
        managers = []
        for uav in range(24):
            dynamic = DynamicGoalModel(
                sinusoid_enabled=mode in ('sin', 'random'),
                random_enabled=mode == 'random', seed=seed, uav_index=uav)
            managers.append(GoalManager(
                environment.initial_goals[uav], environment.changed_goals[uav],
                dynamic_model=dynamic, switch_enabled=mode == 'switch'))
    evaluator = FinalValidationEvaluator(
        planners, environment.obstacles, managers,
        guide_planner if mode == 'switch' else None)
    evaluator.single_planning_duration_seconds = single_planning_duration_seconds
    return evaluator, environment.starts, environment.initial_goals


def dynamic_case(label):
    z = np.linspace(-40., 40., 24)
    starts = np.column_stack((np.zeros(24), np.full(24, 40.), z))
    goals = starts + [1000., 0., 0.]
    if label == 'no-interaction':
        obstacle = Obstacle([500., 40., 500.],
            {'shape': 'cube', 'length': 20., 'width': 20., 'height': 30.,
             'motion': {'model': 'constant_velocity',
                        'velocity': [0., 0., 5.]}}, 100)
    elif label == 'crossing':
        obstacle = Obstacle([100., 40., 75.],
            {'shape': 'rotated_cube', 'length': 20., 'width': 20., 'height': 30.,
             'motion': {'model': 'constant_velocity',
                        'velocity': [0., 0., -15.]}}, 101)
    elif label == 'moving-away':
        obstacle = Obstacle([100., 40., 0.],
            {'shape': 'cube', 'length': 16., 'width': 16., 'height': 25.,
             'motion': {'model': 'constant_velocity',
                        'velocity': [50., 0., 0.]}}, 102)
    else:
        obstacle = Obstacle([120., 40., 0.],
            {'shape': 'cube', 'length': 20., 'width': 25., 'height': 30.,
             'motion': {'model': 'constant_velocity',
                        'velocity': [-10., 0., 0.]}}, 103)
    planners = [RecedingHorizonPlanner(
        GlobalGuideTracker(np.vstack((start, goal)), turn_aware_enabled=True,
                           turn_accel_ref_ratio=.3), [obstacle], goal=goal,
        terminal_capture_speed=10.) for start, goal in zip(starts, goals)]
    return FinalValidationEvaluator(planners, [obstacle]), starts, goals


def run(label, case, cycles):
    evaluator, starts, goals = case
    begin = time.perf_counter()
    result = evaluator.run(label, starts, goals, cycles)
    print('{} success={} cycles={} failure={}/{} arrival={} min={}/{}/{} '
          'updates={} replans={} dyn_reject={} timing={} wall={:.3f}s'.format(
              label, result.success, result.completed_cycles,
              result.first_failure_cycle, result.failed_uav_id,
              result.arrival_count, result.minimum_uav_distance,
              result.minimum_static_clearance, result.minimum_dynamic_clearance,
              result.goal_update_count, result.global_guide_replan_count,
              result.dynamic_rejection_count, result.timing_summary_ms,
              time.perf_counter() - begin))
    if result.failure_snapshot is not None:
        print('{} failure_snapshot={}'.format(label, result.failure_snapshot))
    return result


def main():
    results = []
    results.append(run('fixed-500', formal_case(), 500))
    results.append(run('fixed-1000', formal_case(), 1000))
    for seed in range(10):
        results.append(run('seed-{}'.format(seed),
                           formal_case('random', seed), 100))
    for mode, cycles in (('switch', 500), ('sin', 100), ('random', 100)):
        results.append(run('goal-{}'.format(mode), formal_case(mode, 0), cycles))
    for label in ('no-interaction', 'crossing', 'moving-away', 'close-encounter'):
        results.append(run('dynamic-{}'.format(label), dynamic_case(label), 100))
    results.append(run('stress-conflict', formal_case(), 150))
    benchmark = run('performance-35', formal_case(), 35)
    measured = benchmark.cycle_logs[5:]
    performance = timing_summary([x['total_cycle_time'] for x in measured])
    seed_results = [x for x in results if x.label.startswith('seed-')]
    seed_wall = np.asarray([x.timing_summary_ms['mean'] for x in seed_results])
    overall = all(x.success for x in results) and benchmark.success and \
        performance['p95'] < 200.
    print('MULTI_SEED success_rate={} mean_completion_ms={} std={} worst={} '
          'worst_clearance={} worst_planning_ms={}'.format(
              np.mean([x.success for x in seed_results]), np.mean(seed_wall),
              np.std(seed_wall), np.max(seed_wall),
              min(x.minimum_uav_distance for x in seed_results),
              max(x.timing_summary_ms['max'] for x in seed_results)))
    print('PERFORMANCE warmup=5 measured=30 {}'.format(performance))
    print('FINAL overall={} report={}'.format(
        'PASS' if overall else 'FAIL', report_dict(results + [benchmark])))


if __name__ == '__main__':
    main()
