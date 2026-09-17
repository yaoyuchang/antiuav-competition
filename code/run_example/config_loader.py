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
    """从CSV表格推断动态障碍物运动参数

    注意：运动参数（振幅、周期、方向）在CSV中未明确给出，
    使用与原代码相同的硬编码值

    Args:
        csv_path: CSV文件路径

    Returns:
        dict: 动态障碍物ID到运动参数的映射
    """
    # 这些参数在题目表格中未给出，保持原有设定
    dynamic_motions = {
        10: {'direction': [1.0, 0.0, 0.0], 'amplitude': 150.0, 'period': 80.0},
        11: {'direction': [math.cos(math.radians(30.0)), 0.0,
                           math.sin(math.radians(30.0))],
             'amplitude': 300.0, 'period': 120.0},
        12: {'direction': [0.0, 0.0, 1.0], 'amplitude': 120.0, 'period': 70.0},
    }

    return dynamic_motions


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
