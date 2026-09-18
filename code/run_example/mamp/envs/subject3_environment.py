"""Obstacle environment for competition subject 3.

Coordinates use North-Up-East order ``[x, y, z]``.  All obstacle positions
are measured in metres and all times in seconds.  The implementation is kept
compatible with CPython 3.8.
"""

import math
from pathlib import Path

import numpy as np

from ..agents.obstacle import Obstacle
from ..configs import subject3_config as config


# Compatibility aliases; authoritative values live in subject3_config.py.
OBSTACLE_SAFETY_DISTANCE = config.OBS_SAFE_DISTANCE
UAV_SAFETY_DISTANCE = config.UAV_SAFE_DISTANCE
UAV_COUNT = config.NUM_UAV


def build_subject3_starts(csv_path=None):
    """Return assumed uniformly spaced starts in NUE coordinates.

    The semifinal statement gives only x=0, y=0, z in [-40, 40], not the 24
    individual z values.  Uniform placement is an engineering assumption.

    Args:
        csv_path: Optional path to start_points.csv, if None uses default values
    """
    if csv_path and Path(csv_path).exists():
        import sys
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from config_loader import load_points_from_csv
        return load_points_from_csv(csv_path)

    # Default values
    east_positions = np.linspace(-40.0, 40.0, UAV_COUNT)
    return np.column_stack((np.zeros(UAV_COUNT), np.zeros(UAV_COUNT),
                            east_positions))


def build_subject3_initial_goals(csv_path=None):
    """Return the 24 initial terminal points from table 2.

    Args:
        csv_path: Optional path to goal_points_initial.csv, if None uses default values
    """
    if csv_path and Path(csv_path).exists():
        import sys
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from config_loader import load_points_from_csv
        return load_points_from_csv(csv_path)

    # Default values
    east_positions = np.arange(-100.0, 131.0, 10.0)
    return np.column_stack((np.full(UAV_COUNT, 5000.0),
                            np.full(UAV_COUNT, 50.0), east_positions))


def build_subject3_changed_goals(csv_path=None):
    """Return the 24 replanned terminal points from table 3.

    Args:
        csv_path: Optional path to goal_points_changed.csv, if None uses default values
    """
    if csv_path and Path(csv_path).exists():
        import sys
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from config_loader import load_points_from_csv
        return load_points_from_csv(csv_path)

    # Default values: front 8 unchanged, side 8 moved, rear 8 moved
    unchanged = build_subject3_initial_goals()[:8]
    side_group = np.column_stack((np.full(8, 5000.0), np.full(8, 20.0),
                                  np.arange(-575.0, -539.0, 5.0)))
    rear_group = np.column_stack((np.full(8, 4800.0), np.full(8, 10.0),
                                  np.arange(540.0, 576.0, 5.0)))
    return np.vstack((unchanged, side_group, rear_group))


# number, type, centre x, centre z, size 1, size 2, height
# 默认硬编码值，可通过 load_obstacles_from_config() 从CSV覆盖
SUBJECT3_OBSTACLE_TABLE = (
    (1, 1, 300.0, -267.0, 60.0, 0.0, 50.0),
    (2, 2, 500.0, 200.0, 200.0, 80.0, 75.0),
    (3, 1, 650.0, -50.0, 50.0, 0.0, 30.0),
    (4, 3, 1100.0, -200.0, 400.0, 50.0, 85.0),
    (5, 1, 1100.0, 400.0, 70.0, 0.0, 70.0),
    (6, 2, 1700.0, -500.0, 250.0, 53.0, 60.0),
    (7, 2, 1900.0, 200.0, 100.0, 100.0, 95.0),
    (8, 2, 2000.0, -50.0, 200.0, 80.0, 75.0),
    (9, 2, 3400.0, -433.0, 80.0, 200.0, 95.0),
    (10, 4, 2500.0, -300.0, 200.0, 40.0, 90.0),
    (11, 4, 2800.0, 233.0, 200.0, 40.0, 90.0),
    (12, 4, 1500.0, 300.0, 200.0, 40.0, 90.0),
    (13, 1, 3300.0, 467.0, 60.0, 0.0, 85.0),
    (14, 3, 3500.0, 133.0, 400.0, 50.0, 90.0),
    (15, 1, 3900.0, -200.0, 70.0, 0.0, 60.0),
    (16, 1, 4300.0, 0.0, 60.0, 0.0, 40.0),
    (17, 2, 4600.0, 0.0, 80.0, 267.0, 95.0),
    (18, 1, 4600.0, -433.0, 70.0, 0.0, 55.0),
    (19, 1, 4600.0, 433.0, 70.0, 0.0, 55.0),
)


# ASSUMPTION: the official statement specifies that obstacles 10--12 move but
# does not give their time functions, nor the rotated boxes' orientation.
# These deterministic sinusoidal rules were created for the local demonstration
# environment; they are not official.  Directions are expressed in world NUE
# coordinates, angles in degrees from the x axis.  Defaults only -- override
# through config/dynamic_motions.csv via load_obstacles_from_config(), so that
# every input reaches the planner through the CSV interface.
# "Motion 3" (obstacle 11) follows the rectangle's 30-degree long axis; the
# 300 m amplitude matches the two light-coloured limit positions in the figure.
DEFAULT_ROTATED_ANGLE_DEGREES = 30.0
DYNAMIC_MOTIONS = {
    10: {'direction': [1.0, 0.0, 0.0], 'amplitude': 150.0, 'period': 80.0,
         'angle_degrees': 30.0},
    11: {'direction': [math.cos(math.radians(30.0)), 0.0,
                       math.sin(math.radians(30.0))],
         'amplitude': 300.0, 'period': 120.0, 'angle_degrees': 30.0},
    12: {'direction': [0.0, 0.0, 1.0], 'amplitude': 120.0, 'period': 70.0,
         'angle_degrees': 30.0},
}


def _shape(number, obstacle_type, size1, size2, height):
    common = {'competition_id': number}
    if obstacle_type == 1:
        common.update({'shape': 'cylinder', 'radius': size1, 'height': height})
    elif obstacle_type == 2:
        common.update({'shape': 'cube', 'length': size1, 'width': size2,
                       'height': height})
    elif obstacle_type == 4:
        parameters = DYNAMIC_MOTIONS.get(number)
        if parameters is None:
            raise ValueError(
                'obstacle {} is type 4 (moving rotated box) but has no entry in '
                'dynamic_motions.csv; add one or change its type'.format(number))
        angle = parameters.get('angle_degrees', DEFAULT_ROTATED_ANGLE_DEGREES)
        common.update({'shape': 'rotated_cube', 'length': size1, 'width': size2,
                       'height': height, 'angle': math.radians(angle),
                       'motion': {key: parameters[key] for key in
                                  ('direction', 'amplitude', 'period')}})
    else:
        raise ValueError('unsupported primitive obstacle type: {}'.format(obstacle_type))
    return common


def _append_u_shape(obstacles, number, center_x, center_z, width, arm, height):
    """Append a three-prism U, open toward negative x, to ``obstacles``.

    Opening direction and square outer footprint are inferred from the figure;
    the table only supplies overall width and arm thickness.
    """
    half_offset = (width - arm) / 2.0
    parts = (
        # Closed back and the two arms.  Overlap is intentional and harmless.
        (center_x + half_offset, center_z, arm, width),
        (center_x, center_z - half_offset, width, arm),
        (center_x, center_z + half_offset, width, arm),
    )
    for part_number, (x, z, length, depth) in enumerate(parts):
        shape = {'shape': 'cube', 'length': length, 'width': depth,
                 'height': height, 'competition_id': number,
                 'part': part_number}
        obstacles.append(Obstacle([x, height / 2.0, z], shape, len(obstacles)))


def build_subject3_obstacles():
    """Create all 19 numbered obstacles (23 collision primitives)."""
    obstacles = []
    for number, kind, x, z, size1, size2, height in SUBJECT3_OBSTACLE_TABLE:
        if kind == 3:
            _append_u_shape(obstacles, number, x, z, size1, size2, height)
            continue
        shape = _shape(number, kind, size1, size2, height)
        obstacles.append(Obstacle([x, height / 2.0, z], shape, len(obstacles)))
    return obstacles


class Subject3Environment(object):
    """Own obstacle time, geometry queries and safety-margin checks."""

    def __init__(self, safety_distance=OBSTACLE_SAFETY_DISTANCE,
                 starts_csv=None, initial_goals_csv=None, changed_goals_csv=None):
        """Initialize Subject3 environment.

        Args:
            safety_distance: Obstacle safety margin (meters)
            starts_csv: Path to start_points.csv (optional)
            initial_goals_csv: Path to goal_points_initial.csv (optional)
            changed_goals_csv: Path to goal_points_changed.csv (optional)
        """
        self.safety_distance = float(safety_distance)
        self.time = 0.0
        self.obstacles = build_subject3_obstacles()
        self.starts = build_subject3_starts(starts_csv)
        self.initial_goals = build_subject3_initial_goals(initial_goals_csv)
        self.changed_goals = build_subject3_changed_goals(changed_goals_csv)

    def update(self, time_seconds):
        self.time = float(time_seconds)
        for obstacle in self.obstacles:
            obstacle.update(self.time)

    def step(self, dt):
        self.update(self.time + float(dt))

    def clearance(self, point):
        """Return distance to the nearest solid obstacle surface.

        A point inside an obstacle has zero clearance.
        """
        point = np.asarray(point, dtype='float64')
        return min(math.sqrt(obstacle.distance_sq_to_point(point))
                   for obstacle in self.obstacles)

    def is_safe(self, point, margin=None):
        required = self.safety_distance if margin is None else float(margin)
        return self.clearance(point) >= required

    def colliding_obstacle_numbers(self, point, margin=None):
        required = self.safety_distance if margin is None else float(margin)
        point = np.asarray(point, dtype='float64')
        return sorted(set(
            obstacle.competition_id for obstacle in self.obstacles
            if obstacle.distance_sq_to_point(point) < required ** 2
        ))

    def dynamic_states(self):
        """Return public state for the three moving obstacles."""
        result = {}
        for obstacle in self.obstacles:
            if obstacle.motion:
                result[obstacle.competition_id] = {
                    'position': obstacle.pos_global_frame.copy(),
                    'velocity': obstacle.vel_global_frame.copy(),
                    'angle': obstacle.angle,
                }
        return result


def load_obstacles_from_config(csv_path=None, motions_csv_path=None):
    """从CSV配置文件加载障碍物表格，覆盖全局变量SUBJECT3_OBSTACLE_TABLE

    动态障碍物的运动参数（方向/振幅/周期）和旋转长方体的朝向角度另存于
    dynamic_motions.csv：题面表1的列结构是固定的，塞不进这些工程假设值，
    所以单独一张表，默认在障碍物表同目录下寻找。缺失时保留代码内默认值，
    这样评委只替换题面那张表也能正常跑。

    Args:
        csv_path: 障碍物CSV路径，None时使用默认路径
        motions_csv_path: 动态障碍物参数CSV路径，None时取障碍物表同目录下的
            dynamic_motions.csv
    """
    global SUBJECT3_OBSTACLE_TABLE, DYNAMIC_MOTIONS

    if csv_path is None:
        # 默认路径：从run_example向上找到config目录
        script_dir = Path(__file__).parent.parent.parent.parent
        csv_path = script_dir / 'config' / 'obstacles.csv'

    if not Path(csv_path).exists():
        print(f"警告: 配置文件不存在 {csv_path}，使用默认障碍物配置")
        return

    # 使用config_loader加载
    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from config_loader import load_obstacles_from_csv, load_dynamic_motions_from_csv

    SUBJECT3_OBSTACLE_TABLE = load_obstacles_from_csv(csv_path)
    print(f"已从 {csv_path} 加载 {len(SUBJECT3_OBSTACLE_TABLE)} 个障碍物配置")

    if motions_csv_path is None:
        motions_csv_path = Path(csv_path).parent / 'dynamic_motions.csv'
    if Path(motions_csv_path).exists():
        DYNAMIC_MOTIONS = load_dynamic_motions_from_csv(motions_csv_path)
        print(f"已从 {motions_csv_path} 加载 {len(DYNAMIC_MOTIONS)} 条动态障碍物运动参数")
    else:
        print(f"提示: 未找到 {motions_csv_path}，动态障碍物运动参数使用代码内默认值")


def should_switch_goals(*args, **kwargs):
    """Reserved interface for the ambiguous 3500 m goal-switch condition.

    The statement does not identify whether the trigger is per-UAV, first-UAV,
    all-UAV, or swarm based.  Phase 1 intentionally implements no policy.
    """
    raise NotImplementedError(
        'official 3500 m goal-switch reference has not been clarified')
