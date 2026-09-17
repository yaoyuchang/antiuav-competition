"""Phase-6B frozen-system final validation matrix."""

import time

import numpy as np

from mamp.agents.obstacle import Obstacle
from mamp.envs.subject3_environment import Subject3Environment
from mamp.planners.global_guide import GlobalGuidePlanner, GlobalGuideTracker
from mamp.planners.goal_manager import DynamicGoalModel, GoalManager
from mamp.planners.receding_horizon_planner import RecedingHorizonPlanner
from mamp.simulation.final_validation_evaluator import (
    FinalValidationEvaluator, report_dict, timing_summary)


def formal_case(mode='fixed', seed=0):
    environment = Subject3Environment()
    boundary = np.vstack((environment.starts, environment.initial_goals,
                          environment.changed_goals))
    guide_planner = GlobalGuidePlanner(environment.obstacles, boundary,
                                       vertical_reserve=10.)
    guides = [guide_planner.plan(start, goal) for start, goal in
              zip(environment.starts, environment.initial_goals)]
    planners = [RecedingHorizonPlanner(
        GlobalGuideTracker(guide.waypoints, turn_aware_enabled=True,
                           turn_accel_ref_ratio=.3), environment.obstacles,
        # Dense terminal points are only 5 m apart.  A zero capture speed lets
        # each UAV settle into its 3 m arrival ball without flying through an
        # adjacent UAV that is already holding position.
        goal=goal, terminal_capture_speed=0.)
        for guide, goal in zip(guides, environment.initial_goals)]
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
