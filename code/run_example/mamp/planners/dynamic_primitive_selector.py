"""Dynamic clearance cost and final deterministic candidate ranking."""

from dataclasses import dataclass
import time

import numpy as np

from ..collision.dynamic_primitive_collision import DynamicPrimitiveCollisionChecker
from ..configs import subject3_config as config


@dataclass
class DynamicPrimitiveSelection:
    success: bool
    reason: str
    phase4_selection: object
    planning_time: float
    absolute_times: np.ndarray
    dynamic_safe_mask: np.ndarray
    final_valid_mask: np.ndarray
    min_dynamic_clearance: np.ndarray
    closest_dynamic_obstacle_id: np.ndarray
    time_of_min_dynamic_clearance: np.ndarray
    dynamic_clearance_cost: np.ndarray
    total_cost: np.ndarray
    ranked_indices: np.ndarray
    best_index: object
    dynamic_collision_guard: float
    max_predicted_obstacle_translation_speed: float
    max_predicted_obstacle_rotation_speed: float
    prediction_time: float
    dynamic_collision_time: float
    dynamic_cost_time: float
    dynamic_ranking_time: float
    total_phase5_time: float


class DynamicPrimitiveSelector(object):
    def __init__(self, obstacles):
        self.collision_checker = DynamicPrimitiveCollisionChecker(obstacles)

    def select(self, phase4_selection, planning_time):
        total_start = time.perf_counter()
        collision = self.collision_checker.check(phase4_selection, planning_time)
        final_valid = phase4_selection.valid_mask & collision.dynamic_safe_mask

        cost_start = time.perf_counter()
        deficit = np.maximum(config.DYNAMIC_PREFERRED_CLEARANCE -
                             collision.min_dynamic_clearance, 0.0)
        dynamic_clearance_cost = (deficit / config.DYNAMIC_CLEARANCE_SCALE) ** 2
        total_cost = (phase4_selection.total_cost +
                      config.DYNAMIC_CLEARANCE_COST_WEIGHT * dynamic_clearance_cost)
        cost_time = time.perf_counter() - cost_start

        ranking_start = time.perf_counter()
        valid_indices = np.flatnonzero(final_valid)
        if len(valid_indices):
            order = np.lexsort((valid_indices, total_cost[valid_indices]))
            ranked = valid_indices[order]
            best_index = int(ranked[0])
            success, reason = True, 'success'
        else:
            ranked = np.empty(0, dtype='int64')
            best_index = None
            success, reason = False, 'no dynamically safe primitive'
        ranking_time = time.perf_counter() - ranking_start
        return DynamicPrimitiveSelection(
            success, reason, phase4_selection, float(planning_time),
            collision.absolute_times, collision.dynamic_safe_mask, final_valid,
            collision.min_dynamic_clearance,
            collision.closest_dynamic_obstacle_id,
            collision.time_of_min_dynamic_clearance, dynamic_clearance_cost,
            total_cost, ranked, best_index, collision.dynamic_collision_guard,
            collision.max_predicted_obstacle_translation_speed,
            collision.max_predicted_obstacle_rotation_speed,
            collision.prediction_time, collision.dynamic_collision_time,
            cost_time, ranking_time, time.perf_counter() - total_start)
