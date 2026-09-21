"""标准化规划器运行脚本（带配置文件接口）"""
import argparse
import csv
import json
import sys
import tempfile
from pathlib import Path

# 添加mamp模块到路径
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
from mamp.envs import subject3_environment
from mamp.simulation.mission_metrics import compute_energy_and_smoothness
from config_loader import load_obstacles_from_csv, load_points_from_csv, get_config_path


def _apply_goal_overrides(goals, overrides, label):
    """Return a copy of goals with 1-based UAV coordinate overrides applied."""
    result = np.asarray(goals, dtype='float64').copy()
    for uav, x, y, z in overrides or []:
        if not float(uav).is_integer():
            raise ValueError(f'{label}目标的无人机编号必须是整数，收到 {uav}')
        index = int(uav) - 1
        if index < 0 or index >= len(result):
            raise ValueError(f'{label}目标的无人机编号必须在 1..{len(result)}，收到 {uav}')
        result[index] = [float(x), float(y), float(z)]
        print(f'覆盖{label}目标: UAV {int(uav)} -> ({x}, {y}, {z})')
    return result


def _write_points_csv(csv_path, points):
    """Write point data in the same format accepted by config_loader."""
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['无人机编号', 'X(m)', 'Y(高度)', 'Z(侧向)'])
        for index, (x, y, z) in enumerate(points, start=1):
            writer.writerow([index, x, y, z])


def run_planner_with_config(config_dir, mode='fixed', cycles=2000,
                            output_file=None, seed=0, initial_goal_overrides=None,
                            changed_goal_overrides=None):
    """使用配置文件运行规划器

    Args:
        config_dir: 配置文件目录
        mode: 运行模式 ('fixed', 'switch', 'sin', 'random', 'competition')
        cycles: 最大规划周期数
        output_file: 输出文件路径
        seed: 随机种子
        initial_goal_overrides: 初始目标覆盖列表，每项为 (无人机编号, x, y, z)
        changed_goal_overrides: 切换后目标覆盖列表，每项为 (无人机编号, x, y, z)

    Returns:
        dict: 规划结果和性能指标
    """
    config_path = Path(config_dir)

    # 1. 加载配置文件
    print(f"正在从 {config_dir} 加载配置...")

    # 加载障碍物（会覆盖全局变量SUBJECT3_OBSTACLE_TABLE）
    obstacles_csv = config_path / 'obstacles.csv'
    if obstacles_csv.exists():
        subject3_environment.load_obstacles_from_config(str(obstacles_csv))
    else:
        print(f"警告: 未找到 {obstacles_csv}，使用默认障碍物配置")

    # 加载起点
    starts_csv = config_path / 'start_points.csv'
    if starts_csv.exists():
        starts = load_points_from_csv(str(starts_csv))
        print(f"已加载 {len(starts)} 个起点")
    else:
        starts = subject3_environment.build_subject3_starts()
        print(f"使用默认起点配置")

    # 加载初始终点
    initial_goals_csv = config_path / 'goal_points_initial.csv'
    if initial_goals_csv.exists():
        initial_goals = load_points_from_csv(str(initial_goals_csv))
        print(f"已加载 {len(initial_goals)} 个初始终点")
    else:
        initial_goals = subject3_environment.build_subject3_initial_goals()
        print(f"使用默认初始终点配置")

    # 加载切换后终点
    changed_goals_csv = config_path / 'goal_points_changed.csv'
    if changed_goals_csv.exists():
        changed_goals = load_points_from_csv(str(changed_goals_csv))
        print(f"已加载 {len(changed_goals)} 个切换后终点")
    else:
        changed_goals = subject3_environment.build_subject3_changed_goals()
        print(f"使用默认切换后终点配置")

    initial_goals = _apply_goal_overrides(
        initial_goals, initial_goal_overrides, '初始')
    changed_goals = _apply_goal_overrides(
        changed_goals, changed_goal_overrides, '切换后')

    # 2. 初始化规划器（使用原有的formal_case函数）
    print(f"\n正在初始化规划器 (mode={mode}, seed={seed})...")
    from run_phase6b_final_validation import formal_case

    # competition模式需要特殊处理：先创建switch模式，再启用扰动
    formal_mode = 'switch' if mode == 'competition' else mode

    # 将已加载且可能被命令行覆盖的目标写入临时配置，确保全局路径规划器、
    # 局部规划器和目标管理器使用完全相同的坐标；原始CSV不会被修改。
    with tempfile.TemporaryDirectory(prefix='mamp-goals-') as temp_dir:
        temp_path = Path(temp_dir)
        effective_initial_csv = temp_path / 'goal_points_initial.csv'
        effective_changed_csv = temp_path / 'goal_points_changed.csv'
        _write_points_csv(effective_initial_csv, initial_goals)
        _write_points_csv(effective_changed_csv, changed_goals)
        evaluator, _, _ = formal_case(
            mode=formal_mode,
            seed=seed,
            starts_csv=str(starts_csv) if starts_csv.exists() else None,
            initial_goals_csv=str(effective_initial_csv),
            changed_goals_csv=str(effective_changed_csv)
        )
    # "单次规划时长"官方口径：一次产出全部24条完整路径的用时（L1全局引导），
    # 与下面滚动时域的单周期耗时（timing_summary_ms）是两个不同的指标。
    single_planning_duration_seconds = evaluator.single_planning_duration_seconds

    # 如果是competition模式，额外启用正弦和随机扰动
    if mode == 'competition':
        for manager in evaluator.goal_managers or []:
            manager.dynamic_model.sinusoid_enabled = True
            manager.dynamic_model.random_enabled = True

    # 禁用超时限制（用于动画渲染和评估）
    evaluator.cycle_deadline = float('inf')

    # 3. 运行规划
    print(f"\n开始规划 {cycles} 个周期...")
    print("-" * 60)

    result = evaluator.run(
        label=f'{mode}-config',
        starts=starts,
        goals=initial_goals,
        cycles=cycles,
        stop_on_all_arrived=True
    )

    # 4. 输出结果
    print("\n" + "=" * 60)
    print("规划完成")
    print("=" * 60)
    print(f"成功: {result.success}")
    print(f"单次规划时长(一次产出全部24条完整路径): {single_planning_duration_seconds:.3f} 秒")
    print(f"完成周期: {result.completed_cycles}/{cycles}")
    print(f"任务时间: {result.mission_time:.2f} 秒")
    print(f"到达数量: {result.arrival_count}/24")
    print(f"最小UAV间距: {result.minimum_uav_distance:.3f} 米 (要求≥3.0米)")
    print(f"最小静态障碍物间距: {result.minimum_static_clearance:.3f} 米 (要求≥1.5米)")
    print(f"最小动态障碍物间距: {result.minimum_dynamic_clearance:.3f} 米 (要求≥1.5米)")

    if not result.success:
        print(f"失败原因: {result.reason}")

    # 4b. 计算能耗与平滑性（官方5项指标里另外2项，40%权重）
    mission_metrics = compute_energy_and_smoothness(result.cycle_logs)
    print(f"集群总能量消耗: {mission_metrics['total_energy']:.3f} (Σ∫|需用过载|dt)")
    print(f"路径平滑性(均值): {mission_metrics['mean_path_smoothness']:.6f} (∫κ²ds)")

    # 5. 保存输出
    output_data = {
        'success': result.success,
        'reason': result.reason,
        'single_planning_duration_seconds': single_planning_duration_seconds,
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
        'future_feasibility_rejection_count': result.future_feasibility_rejection_count,
        'fallback_count': result.fallback_count,
        'timing_summary_ms': result.timing_summary_ms,
        # 能耗与平滑性（Σ∫|需用过载|dt，∫κ²ds；平滑性取24机均值，总和供参考）
        'total_energy': mission_metrics['total_energy'],
        'per_uav_energy': mission_metrics['per_uav_energy'],
        'mean_path_smoothness': mission_metrics['mean_path_smoothness'],
        'total_path_smoothness': mission_metrics['total_path_smoothness'],
        'per_uav_smoothness': mission_metrics['per_uav_smoothness'],
        # 元数据
        'metadata': {
            'mode': mode,
            'seed': seed,
            'max_cycles': cycles,
            'num_uavs': len(starts),
            'config_dir': str(config_dir)
        }
    }

    if output_file:
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2, ensure_ascii=False)

        print(f"\n结果已保存到: {output_path}")

    return output_data


def main():
    parser = argparse.ArgumentParser(
        description='运行科目3无人机路径规划（从配置文件加载）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
运行模式:
  fixed       - 固定目标点（无干扰）
  switch      - 3500m目标切换
  sin         - 正弦扰动目标
  random      - 随机噪声目标
  competition - 综合模式（切换+正弦+随机）

示例用法:
  # 使用默认配置运行
  python run_with_config.py --config-dir ../config --mode competition

  # 保存结果
  python run_with_config.py --config-dir ../config --mode fixed --output output/result.json

  # 指定周期数和随机种子
  python run_with_config.py --config-dir ../config --cycles 3000 --seed 42
        """
    )

    parser.add_argument('--config-dir', default='../config',
                       help='配置文件目录路径 (默认: ../config)')
    parser.add_argument('--mode', default='competition',
                       choices=['fixed', 'switch', 'sin', 'random', 'competition'],
                       help='运行模式 (默认: competition)')
    parser.add_argument('--cycles', type=int, default=2000,
                       help='最大规划周期数 (默认: 2000)')
    parser.add_argument('--output', default='output/trajectory_output.json',
                       help='输出文件路径 (默认: output/trajectory_output.json)')
    parser.add_argument('--seed', type=int, default=0,
                       help='随机种子 (默认: 0)')
    parser.add_argument(
        '--initial-goal', action='append', nargs=4, type=float,
        metavar=('UAV', 'X', 'Y', 'Z'),
        help='覆盖初始目标点，可重复使用；UAV编号从1开始')
    parser.add_argument(
        '--changed-goal', action='append', nargs=4, type=float,
        metavar=('UAV', 'X', 'Y', 'Z'),
        help='覆盖切换后目标点，可重复使用；UAV编号从1开始')

    args = parser.parse_args()

    # 运行规划
    result = run_planner_with_config(
        config_dir=args.config_dir,
        mode=args.mode,
        cycles=args.cycles,
        output_file=args.output,
        seed=args.seed,
        initial_goal_overrides=args.initial_goal,
        changed_goal_overrides=args.changed_goal
    )

    # 返回成功/失败状态
    sys.exit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
