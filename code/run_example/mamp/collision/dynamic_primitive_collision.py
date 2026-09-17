"""Absolute-time, candidate-by-time dynamic obstacle collision checks."""

from dataclasses import dataclass
import math
import time

import numpy as np

from ..configs import subject3_config as config


@dataclass
class DynamicCollisionResult:
    planning_time: float
    absolute_times: np.ndarray
    dynamic_safe_mask: np.ndarray
    min_dynamic_clearance: np.ndarray
    closest_dynamic_obstacle_id: np.ndarray
    time_of_min_dynamic_clearance: np.ndarray
    dynamic_collision_guard: float
    max_predicted_obstacle_translation_speed: float
    max_predicted_obstacle_rotation_speed: float
    prediction_time: float
    dynamic_collision_time: float


def distance_to_predicted_obstacle(positions, obstacle, centers, yaw):
    """Same-time distances for positions (Nc,Nt,3) and poses (Nt,...)."""
    relative = positions - centers[None, :, :]
    if obstacle.shape == 'sphere':
        return np.maximum(0.0, np.linalg.norm(relative, axis=2) - obstacle.radius)
    if obstacle.shape == 'cylinder':
        radial = np.sqrt(relative[:, :, 0] ** 2 + relative[:, :, 2] ** 2)
        radial_gap = np.maximum(0.0, radial - obstacle.cylinder_radius)
        vertical_gap = np.maximum(
            0.0, np.abs(relative[:, :, 1]) - obstacle.height / 2.0)
        return np.sqrt(radial_gap ** 2 + vertical_gap ** 2)
    cosine = np.cos(yaw)[None, :]
    sine = np.sin(yaw)[None, :]
    local_x = cosine * relative[:, :, 0] + sine * relative[:, :, 2]
    local_z = -sine * relative[:, :, 0] + cosine * relative[:, :, 2]
    gap_x = np.maximum(0.0, np.abs(local_x) - obstacle.length / 2.0)
    gap_y = np.maximum(0.0, np.abs(relative[:, :, 1]) - obstacle.height / 2.0)
    gap_z = np.maximum(0.0, np.abs(local_z) - obstacle.width / 2.0)
    return np.sqrt(gap_x ** 2 + gap_y ** 2 + gap_z ** 2)


class DynamicPrimitiveCollisionChecker(object):
    """Filter Phase-4 candidates against predicted dynamic obstacle poses."""

    def __init__(self, obstacles, constraint_epsilon=None):
        self.dynamic_obstacles = tuple(obstacle for obstacle in obstacles
                                       if not obstacle.is_static)
        self.constraint_epsilon = float(
            config.PRIMITIVE_CONSTRAINT_EPS if constraint_epsilon is None
            else constraint_epsilon)

    def check(self, phase4_selection, planning_time):
        planning_time = float(planning_time)
        if not math.isfinite(planning_time):
            raise ValueError('planning_time must be finite')
        validation = phase4_selection.validation
        relative_times = validation.validation_times
        absolute_times = planning_time + relative_times
        primitive_count = len(phase4_selection.batch.positions)
        dynamic_safe = np.ones(primitive_count, dtype=bool)
        min_clearance = np.full(primitive_count, np.inf, dtype='float64')
        closest_id = np.full(primitive_count, -1, dtype='int64')
        time_of_min = np.full(primitive_count, np.nan, dtype='float64')
        candidate_indices = phase4_selection.ranked_indices

        prediction_start = time.perf_counter()
        predictions = []
        max_translation_speed = 0.0
        max_rotation_speed = 0.0
        for obstacle in self.dynamic_obstacles:
            centers, yaw = obstacle.predict_pose(absolute_times)
            yaw_rate = obstacle.predict_yaw_rate(absolute_times)
            # Prefer the motion model's analytic speed bound.  The sampled
            # velocity remains part of the auditable prediction interface.
            translation_speed = obstacle.translation_speed_bound()
            if obstacle.shape in ('cube', 'rotated_cube'):
                box_radius = math.sqrt((obstacle.length / 2.0) ** 2 +
                                       (obstacle.width / 2.0) ** 2)
                rotation_speed = float(np.max(np.abs(yaw_rate))) * box_radius
            else:
                rotation_speed = 0.0
            max_translation_speed = max(max_translation_speed, translation_speed)
            max_rotation_speed = max(max_rotation_speed, rotation_speed)
            predictions.append((obstacle, centers, np.asarray(yaw, dtype='float64')))
        prediction_time = time.perf_counter() - prediction_start
        guard = 0.5 * (config.V_MAX + max_translation_speed +
                       max_rotation_speed) * config.VALIDATION_DT

        collision_start = time.perf_counter()
        if len(candidate_indices):
            candidate_positions = validation.positions[candidate_indices]
            candidate_minimum = np.full(len(candidate_indices), np.inf)
            candidate_obstacle = np.full(len(candidate_indices), -1, dtype='int64')
            candidate_time = np.full(len(candidate_indices), np.nan)
            for obstacle, centers, yaw in predictions:
                distances = distance_to_predicted_obstacle(
                    candidate_positions, obstacle, centers, yaw)
                time_indices = np.argmin(distances, axis=1)
                row_indices = np.arange(len(candidate_indices))
                obstacle_minimum = distances[row_indices, time_indices]
                improved = obstacle_minimum < candidate_minimum
                candidate_minimum[improved] = obstacle_minimum[improved]
                candidate_obstacle[improved] = int(obstacle.competition_id)
                candidate_time[improved] = relative_times[time_indices[improved]]
            min_clearance[candidate_indices] = candidate_minimum
            closest_id[candidate_indices] = candidate_obstacle
            time_of_min[candidate_indices] = candidate_time
            dynamic_safe[candidate_indices] = (
                candidate_minimum >= config.OBS_SAFE_DISTANCE + guard -
                self.constraint_epsilon)
        dynamic_safe[~phase4_selection.valid_mask] = False
        collision_time = time.perf_counter() - collision_start
        return DynamicCollisionResult(
            planning_time, absolute_times, dynamic_safe, min_clearance,
            closest_id, time_of_min, guard, max_translation_speed,
            max_rotation_speed, prediction_time, collision_time)
