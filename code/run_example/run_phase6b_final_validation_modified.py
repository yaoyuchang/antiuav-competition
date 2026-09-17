"""Phase-6B frozen-system final validation matrix.

修改版：支持从CSV读取配置并保存JSON输出
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from mamp.agents.obstacle import Obstacle
from mamp.envs.subject3_environment import Subject3Environment
from mamp.planners.global_guide import GlobalGuidePlanner, GlobalGuideTracker
from mamp.planners.goal_manager import DynamicGoalModel, GoalManager
from mamp.planners.receding_horizon_planner import RecedingHorizonPlanner
from mamp.simulation.final_validation_evaluator import (
    FinalValidationEvaluator, report_dict, timing_summary)


def formal_case(mode='fixed', seed=0, starts_csv=None, initial_goals_csv=None,
                changed_goals_csv=None):
    """Create formal validation case with optional CSV configuration.

    Args:
        mode: Running mode ('fixed', 'switch', 'sin', 'random')
        seed: Random seed
        starts_csv: Path to start_points.csv (optional)
        initial_goals_csv: Path to goal_points_initial.csv (optional)
        changed_goals_csv: Path to goal_points_changed.csv (optional)
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
    guides = [guide_planner.plan(start, goal) for start, goal in
              zip(environment.starts, environment.initial_goals)]
    planners = [RecedingHorizonPlanner(
        GlobalGuideTracker(guide.waypoints, turn_aware_enabled=True,
                           turn_accel_ref_ratio=.3), environment.obstacles,
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


def main_original():
    """原有的批量测试功能"""
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
    print('overall={} mean_seed_wall={:.2f}ms p95={:.2f}ms'.format(
        overall, np.mean(seed_wall), performance['p95']))


def main_with_config():
    """新增：从CSV读取配置并保存JSON（比赛接口）"""
    parser = argparse.ArgumentParser(description='运行科目3规划（支持CSV配置）')
    parser.add_argument('--config-dir', default='../config', help='配置文件目录')
    parser.add_argument('--mode', default='competition',
                       choices=['fixed', 'switch', 'sin', 'random', 'competition'])
    parser.add_argument('--cycles', type=int, default=2000, help='最大周期数')
    parser.add_argument('--output', default='output/trajectory_output.json', help='输出JSON文件')
    parser.add_argument('--seed', type=int, default=0, help='随机种子')
    parser.add_argument('--test-mode', action='store_true', help='运行原有的批量测试')

    args = parser.parse_args()

    # 如果指定 --test-mode，运行原有测试
    if args.test_mode:
        main_original()
        return

    # 否则运行比赛接口模式
    config_path = Path(args.config_dir)

    # 准备CSV路径
    starts_csv = str(config_path / 'start_points.csv') if (config_path / 'start_points.csv').exists() else None
    initial_goals_csv = str(config_path / 'goal_points_initial.csv') if (config_path / 'goal_points_initial.csv').exists() else None
    changed_goals_csv = str(config_path / 'goal_points_changed.csv') if (config_path / 'goal_points_changed.csv').exists() else None

    print(f"从 {args.config_dir} 加载配置...")
    if starts_csv:
        print(f"  起点: {starts_csv}")
    if initial_goals_csv:
        print(f"  初始终点: {initial_goals_csv}")
    if changed_goals_csv:
        print(f"  切换终点: {changed_goals_csv}")

    # 创建场景
    case = formal_case(
        mode=args.mode,
        seed=args.seed,
        starts_csv=starts_csv,
        initial_goals_csv=initial_goals_csv,
        changed_goals_csv=changed_goals_csv
    )

    # 运行规划
    print(f"\n开始规划 (mode={args.mode}, cycles={args.cycles})...")
    result = run(f'{args.mode}-config', case, args.cycles)

    # 保存JSON输出
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    output_data = {
        'success': result.success,
        'reason': result.reason,
        'completed_cycles': result.completed_cycles,
        'mission_time': result.mission_time,
        'arrival_count': result.arrival_count,
        'minimum_uav_distance': result.minimum_uav_distance,
        'minimum_static_clearance': result.minimum_static_clearance,
        'minimum_dynamic_clearance': result.minimum_dynamic_clearance,
        'uav_violation_count': result.uav_violation_count,
        'static_violation_count': result.static_violation_count,
        'dynamic_violation_count': result.dynamic_violation_count,
        'goal_update_count': result.goal_update_count,
        'global_guide_replan_count': result.global_guide_replan_count,
        'dynamic_rejection_count': result.dynamic_rejection_count,
        'k3_solved_count': result.k3_solved_count,
        'k5_activation_count': result.k5_activation_count,
        'joint_search_count': result.joint_search_count,
        'fallback_count': result.fallback_count,
        'timing_summary_ms': result.timing_summary_ms,
        'metadata': {
            'mode': args.mode,
            'seed': args.seed,
            'max_cycles': args.cycles,
            'config_dir': args.config_dir
        }
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"\n结果已保存到: {output_path}")


if __name__ == '__main__':
    # 检查命令行参数，决定运行哪个模式
    import sys
    if len(sys.argv) > 1:
        main_with_config()  # 有参数 -> 比赛接口模式
    else:
        main_original()     # 无参数 -> 原有测试模式
