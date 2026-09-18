"""从CSV表格加载配置的简单接口"""
import csv
import math
import numpy as np
from pathlib import Path


def load_obstacles_from_csv(csv_path):
    """从CSV表格加载障碍物配置，返回SUBJECT3_OBSTACLE_TABLE格式的元组

    Args:
        csv_path: CSV文件路径

    Returns:
        tuple: 与SUBJECT3_OBSTACLE_TABLE相同格式的元组
            (编号, 类型, 中心x, 中心z, 尺寸1, 尺寸2, 高度)
    """
    obstacles = []

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            obstacle = (
                int(row['编号']),
                int(row['类型']),
                float(row['中心x']),
                float(row['中心z']),
                float(row['尺寸1']),
                float(row['尺寸2']),
                float(row['高度'])
            )
            obstacles.append(obstacle)

    return tuple(obstacles)


def load_dynamic_motions_from_csv(csv_path):
    """从CSV表格加载动态障碍物（类型4）的运动与朝向参数

    题面只说明10/11/12号障碍物会运动，没有给出具体的运动方向、振幅、周期，
    也没有给出旋转长方体的朝向角度，这些都是工程假设值。此前它们硬编码在
    subject3_environment.py 里，现在改为从本表读取，使得全部输入都走接口。

    表头：编号,运动方向角度,振幅,周期,朝向角度
    两个角度都是与 x 轴的夹角（度），运动方向在水平面内，
    换算成世界NUE坐标的单位向量 [cos, 0, sin]——直接存角度而不是存分量，
    一是可读可改（题面图里给的本来就是"沿30度长轴"这种描述），
    二是代码里现算 cos/sin 与原先硬编码的写法得到完全相同的浮点数，
    不会因为手抄小数位引入精度扰动。

    Args:
        csv_path: CSV文件路径

    Returns:
        dict: 障碍物编号 -> {'direction', 'amplitude', 'period', 'angle_degrees'}
    """
    motions = {}

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            heading = math.radians(float(row['运动方向角度']))
            entry = {
                'direction': [math.cos(heading), 0.0, math.sin(heading)],
                'amplitude': float(row['振幅']),
                'period': float(row['周期']),
            }
            # 朝向角度列可选：缺列或留空时由调用方回退到默认朝向
            angle = row.get('朝向角度')
            if angle not in (None, ''):
                entry['angle_degrees'] = float(angle)
            motions[int(row['编号'])] = entry

    return motions


def load_points_from_csv(csv_path):
    """从CSV表格加载起点或终点坐标

    Args:
        csv_path: CSV文件路径

    Returns:
        np.ndarray: (N, 3) 形状的坐标数组 [x, y, z]
    """
    points = []

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # 题目使用 X, Y(高度), Z(侧向)
            # 代码使用 [x, y, z] = [North, Up, East]
            point = [
                float(row['X(m)']),
                float(row['Y(高度)']),
                float(row['Z(侧向)'])
            ]
            points.append(point)

    return np.array(points, dtype='float64')


def get_config_path(relative_path):
    """获取配置文件的绝对路径

    Args:
        relative_path: 相对于config目录的路径

    Returns:
        Path: 绝对路径
    """
    # 从 run_example/ 目录向上找到 code/config/
    script_dir = Path(__file__).parent
    config_dir = script_dir.parent / 'config'
    return config_dir / relative_path


if __name__ == '__main__':
    # 测试加载
    obstacles = load_obstacles_from_csv(get_config_path('obstacles.csv'))
    print(f"加载了 {len(obstacles)} 个障碍物")
    print(f"第一个障碍物: {obstacles[0]}")

    starts = load_points_from_csv(get_config_path('start_points.csv'))
    print(f"\n加载了 {len(starts)} 个起点")
    print(f"第一个起点: {starts[0]}")

    goals = load_points_from_csv(get_config_path('goal_points_initial.csv'))
    print(f"\n加载了 {len(goals)} 个终点")
    print(f"第一个终点: {goals[0]}")
