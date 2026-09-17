"""独立的指标计算脚本，用于评估规划结果

与 run_with_config.py（规划器）完全解耦：只读取 trajectory_output.json，
不依赖 mamp/ 里的任何规划代码，符合"模块化封装"里"规划器"和"评分脚本"
必须是两个独立程序的要求。

评分口径对齐复赛科目三题面公布的官方 5 项指标与权重：
    单次规划时长 30% / 无人机抵达时长 30% / 集群总能量消耗 20%
    / 路径安全性 10% / 路径平滑性 10%

题面只给出了权重，没有给出每项指标"满分/零分"对应的具体数值基准，
下面 SCORE_REFERENCES 里的阈值是工程假设（部分有依据，见注释），
不是官方数值。评委或队内如有更准确的基准，直接改这里的常量即可，
不需要改评分逻辑。
"""
import json
import argparse
import numpy as np
from pathlib import Path


# 每项指标的满分/零分参考基准。得分在两点间线性插值，超出范围截断到[0,100]。
SCORE_REFERENCES = {
    # 满分：一次产出全部24条完整路径耗时 <= 1 秒（接近实时）。
    # 零分：>= 20 秒。均为工程假设，题面未给出基准。
    'planning_duration_full_s': 1.0,
    'planning_duration_zero_s': 20.0,
    # 满分基准取自 final_version_架构说明.md 第七节给出的理论下界估算
    # （约103秒，直线飞行时间量级），零分取其3倍，因为题面没有给上界。
    'arrival_time_full_s': 103.0,
    'arrival_time_zero_s': 300.0,
    # 能耗（Σ∫|需用过载|dt）和平滑性（均值∫κ²ds）的基准锚定在一次完整成功任务的
    # 实测值上（competition模式、种子0、614周期、24/24到达）：
    #   集群总能耗 617.7，路径平滑性(均值) 1.874
    # 满分/零分取该实测值的上下区间，使得基准运行落在中段而不是贴着0分或100分。
    # 题面没有给出官方基准，这仍是工程假设，但已由实测定标，不再是凭空取值。
    'energy_full': 400.0,
    'energy_zero': 1000.0,
    'smoothness_full': 0.5,
    'smoothness_zero': 5.0,
}

OFFICIAL_WEIGHTS = {
    '规划时长': 0.30,
    '抵达时长': 0.30,
    '能耗': 0.20,
    '安全性': 0.10,
    '平滑性': 0.10,
}


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


def _linear_score(value, full_score_at, zero_score_at):
    """越小越好的指标：value<=full_score_at 记100分，>=zero_score_at 记0分。"""
    if zero_score_at <= full_score_at:
        raise ValueError('zero_score_at 必须大于 full_score_at')
    ratio = (zero_score_at - value) / (zero_score_at - full_score_at)
    return float(np.clip(ratio, 0.0, 1.0) * 100.0)


def calculate_metrics(trajectory_data):
    """计算比赛要求的性能指标

    包含官方5项评分指标（规划时长/抵达时长/能耗/安全性/平滑性）
    以及若干规划过程的诊断字段（违规次数、回退次数等，仅供调试参考，
    不参与总分计算）。

    Args:
        trajectory_data: 轨迹数据字典

    Returns:
        dict: 计算得到的指标
    """
    metrics = {}

    # 基础指标（从输出中提取）
    metrics['成功'] = trajectory_data.get('success', False)
    metrics['单次规划时长(秒)'] = trajectory_data.get(
        'single_planning_duration_seconds', float('nan'))
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

    # 能耗与平滑性（官方另外2项指标）
    metrics['集群总能量消耗'] = trajectory_data.get('total_energy', float('nan'))
    metrics['路径平滑性(均值)'] = trajectory_data.get('mean_path_smoothness', float('nan'))
    metrics['路径平滑性(总和,参考)'] = trajectory_data.get('total_path_smoothness', float('nan'))

    # 规划过程诊断指标（不参与总分，仅供调试/答辩参考）
    metrics['目标更新次数'] = trajectory_data.get('goal_update_count', 0)
    metrics['全局引导重规划次数'] = trajectory_data.get('global_guide_replan_count', 0)
    metrics['动态排斥次数'] = trajectory_data.get('dynamic_rejection_count', 0)
    metrics['K3求解次数'] = trajectory_data.get('k3_solved_count', 0)
    metrics['K5激活次数'] = trajectory_data.get('k5_activation_count', 0)
    metrics['联合搜索次数'] = trajectory_data.get('joint_search_count', 0)
    metrics['回退次数'] = trajectory_data.get('fallback_count', 0)

    # 违规判定（比赛标准，用于安全性得分）
    uav_safe_threshold = 3.0  # 米
    obs_safe_threshold = 1.5  # 米

    metrics['UAV间距违规'] = metrics['最小UAV间距(米)'] < uav_safe_threshold
    metrics['静态障碍物违规'] = metrics['最小静态障碍物间距(米)'] < obs_safe_threshold
    metrics['动态障碍物违规'] = metrics['最小动态障碍物间距(米)'] < obs_safe_threshold

    # ------------------------------------------------------------------
    # 官方5项指标的折算得分与加权总分
    # ------------------------------------------------------------------
    safety_score = 100.0
    if metrics['UAV间距违规']:
        safety_score -= 30.0
    if metrics['静态障碍物违规']:
        safety_score -= 30.0
    if metrics['动态障碍物违规']:
        safety_score -= 20.0
    safety_score = max(0.0, safety_score)

    planning_duration_score = _linear_score(
        metrics['单次规划时长(秒)'], SCORE_REFERENCES['planning_duration_full_s'],
        SCORE_REFERENCES['planning_duration_zero_s'])
    arrival_time_score = _linear_score(
        metrics['任务完成时间(秒)'], SCORE_REFERENCES['arrival_time_full_s'],
        SCORE_REFERENCES['arrival_time_zero_s'])
    energy_score = _linear_score(
        metrics['集群总能量消耗'], SCORE_REFERENCES['energy_full'],
        SCORE_REFERENCES['energy_zero'])
    smoothness_score = _linear_score(
        metrics['路径平滑性(均值)'], SCORE_REFERENCES['smoothness_full'],
        SCORE_REFERENCES['smoothness_zero'])

    if not metrics['成功']:
        # 未成功完成任务时，抵达时长/能耗/平滑性缺乏完整轨迹意义，直接清零。
        arrival_time_score = energy_score = smoothness_score = 0.0

    metrics['规划时长得分'] = planning_duration_score
    metrics['抵达时长得分'] = arrival_time_score
    metrics['能耗得分'] = energy_score
    metrics['安全性得分'] = safety_score
    metrics['平滑性得分'] = smoothness_score

    metrics['总分'] = (
        OFFICIAL_WEIGHTS['规划时长'] * planning_duration_score +
        OFFICIAL_WEIGHTS['抵达时长'] * arrival_time_score +
        OFFICIAL_WEIGHTS['能耗'] * energy_score +
        OFFICIAL_WEIGHTS['安全性'] * safety_score +
        OFFICIAL_WEIGHTS['平滑性'] * smoothness_score
    )

    return metrics


def print_metrics(metrics):
    """格式化打印指标"""
    print("\n" + "=" * 60)
    print("科目3 无人机路径规划性能指标")
    print("=" * 60)

    print("\n【基础指标】")
    print(f"  成功:                    {metrics['成功']}")
    print(f"  单次规划时长:            {metrics['单次规划时长(秒)']:.3f} 秒")
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

    print("\n【能耗与平滑性】")
    print(f"  集群总能量消耗:          {metrics['集群总能量消耗']:.3f} (Σ∫|需用过载|dt)")
    print(f"  路径平滑性(均值):        {metrics['路径平滑性(均值)']:.6f} (∫κ²ds，评分采用此值)")
    print(f"  路径平滑性(总和,参考):   {metrics['路径平滑性(总和,参考)']:.6f}")

    print("\n【规划过程诊断（不计入总分）】")
    print(f"  目标更新次数:            {metrics['目标更新次数']}")
    print(f"  全局引导重规划次数:      {metrics['全局引导重规划次数']}")
    print(f"  动态排斥次数:            {metrics['动态排斥次数']}")
    print(f"  K3求解次数:              {metrics['K3求解次数']}")
    print(f"  K5激活次数:              {metrics['K5激活次数']}")
    print(f"  联合搜索次数:            {metrics['联合搜索次数']}")
    print(f"  回退次数:                {metrics['回退次数']}")

    print("\n【官方5项指标评分】")
    print(f"  单次规划时长 (30%):      {metrics['规划时长得分']:.1f}/100")
    print(f"  无人机抵达时长 (30%):    {metrics['抵达时长得分']:.1f}/100")
    print(f"  集群总能量消耗 (20%):    {metrics['能耗得分']:.1f}/100")
    print(f"  路径安全性 (10%):        {metrics['安全性得分']:.1f}/100")
    print(f"  路径平滑性 (10%):        {metrics['平滑性得分']:.1f}/100")
    print(f"  ------------------------------------------")
    print(f"  加权总分:                {metrics['总分']:.1f}/100")

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
