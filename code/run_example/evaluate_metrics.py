"""独立的指标计算脚本，用于评估规划结果"""
import json
import argparse
import numpy as np
from pathlib import Path


def load_trajectory_output(filepath):
    """加载轨迹输出文件

    Args:
        filepath: trajectory_output.json路径

    Returns:
        dict: 包含轨迹数据和元数据
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data


def calculate_metrics(trajectory_data):
    """计算比赛要求的性能指标

    根据题目要求计算：
    1. 任务完成时间
    2. 最小UAV间距
    3. 最小障碍物间距
    4. 碰撞次数
    5. 到达数量

    Args:
        trajectory_data: 轨迹数据字典

    Returns:
        dict: 计算得到的指标
    """
    metrics = {}

    # 基础指标（从输出中提取）
    metrics['成功'] = trajectory_data.get('success', False)
    metrics['任务完成时间(秒)'] = trajectory_data.get('mission_time', 0.0)
    metrics['完成周期数'] = trajectory_data.get('completed_cycles', 0)
    metrics['到达UAV数量'] = trajectory_data.get('arrival_count', 0)

    # 安全距离指标
    metrics['最小UAV间距(米)'] = trajectory_data.get('minimum_uav_distance', float('inf'))
    metrics['最小静态障碍物间距(米)'] = trajectory_data.get('minimum_static_clearance', float('inf'))
    metrics['最小动态障碍物间距(米)'] = trajectory_data.get('minimum_dynamic_clearance', float('inf'))

    # 违规统计
    metrics['UAV碰撞次数'] = trajectory_data.get('uav_violation_count', 0)
    metrics['静态障碍物碰撞次数'] = trajectory_data.get('static_violation_count', 0)
    metrics['动态障碍物碰撞次数'] = trajectory_data.get('dynamic_violation_count', 0)

    # 规划性能指标
    metrics['目标更新次数'] = trajectory_data.get('goal_update_count', 0)
    metrics['全局引导重规划次数'] = trajectory_data.get('global_guide_replan_count', 0)
    metrics['动态排斥次数'] = trajectory_data.get('dynamic_rejection_count', 0)
    metrics['K3求解次数'] = trajectory_data.get('k3_solved_count', 0)
    metrics['K5激活次数'] = trajectory_data.get('k5_activation_count', 0)
    metrics['联合搜索次数'] = trajectory_data.get('joint_search_count', 0)
    metrics['回退次数'] = trajectory_data.get('fallback_count', 0)

    # 计算违规判定（比赛标准）
    uav_safe_threshold = 3.0  # 米
    obs_safe_threshold = 1.5  # 米

    metrics['UAV间距违规'] = metrics['最小UAV间距(米)'] < uav_safe_threshold
    metrics['静态障碍物违规'] = metrics['最小静态障碍物间距(米)'] < obs_safe_threshold
    metrics['动态障碍物违规'] = metrics['最小动态障碍物间距(米)'] < obs_safe_threshold

    # 总体评分（简单示例）
    metrics['安全性得分'] = 100.0
    if metrics['UAV间距违规']:
        metrics['安全性得分'] -= 30.0
    if metrics['静态障碍物违规']:
        metrics['安全性得分'] -= 30.0
    if metrics['动态障碍物违规']:
        metrics['安全性得分'] -= 20.0

    metrics['效率得分'] = max(0, 100 - metrics['任务完成时间(秒)'] / 2.0)

    metrics['总分'] = (metrics['安全性得分'] * 0.6 +
                      metrics['效率得分'] * 0.4)

    return metrics


def print_metrics(metrics):
    """格式化打印指标"""
    print("\n" + "=" * 60)
    print("科目3 无人机路径规划性能指标")
    print("=" * 60)

    print("\n【基础指标】")
    print(f"  成功:                    {metrics['成功']}")
    print(f"  任务完成时间:            {metrics['任务完成时间(秒)']:.2f} 秒")
    print(f"  完成周期数:              {metrics['完成周期数']}")
    print(f"  到达UAV数量:             {metrics['到达UAV数量']}/24")

    print("\n【安全距离指标】")
    print(f"  最小UAV间距:             {metrics['最小UAV间距(米)']:.3f} 米 "
          f"({'违规' if metrics['UAV间距违规'] else '合格'}, 要求≥3.0米)")
    print(f"  最小静态障碍物间距:      {metrics['最小静态障碍物间距(米)']:.3f} 米 "
          f"({'违规' if metrics['静态障碍物违规'] else '合格'}, 要求≥1.5米)")
    print(f"  最小动态障碍物间距:      {metrics['最小动态障碍物间距(米)']:.3f} 米 "
          f"({'违规' if metrics['动态障碍物违规'] else '合格'}, 要求≥1.5米)")

    print("\n【碰撞统计】")
    print(f"  UAV碰撞次数:             {metrics['UAV碰撞次数']}")
    print(f"  静态障碍物碰撞次数:      {metrics['静态障碍物碰撞次数']}")
    print(f"  动态障碍物碰撞次数:      {metrics['动态障碍物碰撞次数']}")

    print("\n【规划性能】")
    print(f"  目标更新次数:            {metrics['目标更新次数']}")
    print(f"  全局引导重规划次数:      {metrics['全局引导重规划次数']}")
    print(f"  动态排斥次数:            {metrics['动态排斥次数']}")
    print(f"  K3求解次数:              {metrics['K3求解次数']}")
    print(f"  K5激活次数:              {metrics['K5激活次数']}")
    print(f"  联合搜索次数:            {metrics['联合搜索次数']}")
    print(f"  回退次数:                {metrics['回退次数']}")

    print("\n【评分】")
    print(f"  安全性得分:              {metrics['安全性得分']:.1f}/100")
    print(f"  效率得分:                {metrics['效率得分']:.1f}/100")
    print(f"  总分:                    {metrics['总分']:.1f}/100")

    print("=" * 60 + "\n")


def save_metrics(metrics, filepath):
    """保存指标到JSON文件"""
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print(f"指标已保存到: {filepath}")


def main():
    parser = argparse.ArgumentParser(
        description='计算科目3路径规划性能指标',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python evaluate_metrics.py --input output/trajectory_output.json
  python evaluate_metrics.py --input output/trajectory_output.json --output metrics.json
        """
    )

    parser.add_argument('--input', required=True,
                       help='轨迹输出文件路径 (trajectory_output.json)')
    parser.add_argument('--output', default=None,
                       help='指标输出文件路径 (默认: 输入文件同目录下的metrics.json)')

    args = parser.parse_args()

    # 加载轨迹数据
    print(f"正在加载轨迹数据: {args.input}")
    trajectory_data = load_trajectory_output(args.input)

    # 计算指标
    print("正在计算性能指标...")
    metrics = calculate_metrics(trajectory_data)

    # 打印指标
    print_metrics(metrics)

    # 保存指标
    if args.output is None:
        output_path = Path(args.input).parent / 'metrics.json'
    else:
        output_path = Path(args.output)

    save_metrics(metrics, str(output_path))


if __name__ == '__main__':
    main()
