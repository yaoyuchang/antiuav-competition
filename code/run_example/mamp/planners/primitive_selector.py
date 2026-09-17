"""Static-safe primitive costs and deterministic ranking."""

from dataclasses import dataclass
import time

import numpy as np

from ..collision.static_primitive_collision import StaticPrimitiveCollisionChecker
from ..configs import subject3_config as config


@dataclass
class StaticPrimitiveSelection:
    success: bool
    reason: str
    batch: object
    validation: object
    fine_dynamic_mask: np.ndarray
    static_safe_mask: np.ndarray
    valid_mask: np.ndarray
    min_static_clearance: np.ndarray
    progress: np.ndarray
    progress_cost: np.ndarray
    curvature_cost: np.ndarray
    clearance_cost: np.ndarray
    offset_cost: np.ndarray
    effort_proxy: np.ndarray
    total_cost: np.ndarray
    ranked_indices: np.ndarray
    best_index: object
    validation_times: np.ndarray
    fine_validation_time: float
    static_collision_time: float
    cost_time: float
    ranking_time: float
    total_selection_time: float


class StaticPrimitiveSelector(object):
    """Retain and rank all fine-dynamically feasible, statically safe candidates."""

    def __init__(self, obstacles, **collision_options):
        self.collision_checker = StaticPrimitiveCollisionChecker(
            obstacles, **collision_options)

    @staticmethod
    def _normalized_direction(global_direction):
        direction = np.asarray(global_direction, dtype='float64')
        if direction.shape != (3,):
            raise ValueError('global_direction must have shape (3,)')
        norm = float(np.linalg.norm(direction))
        if norm <= config.PRIMITIVE_DIRECTION_EPS:
            raise ValueError('global_direction must be nonzero')
        return direction / norm

    def select(self, batch, p0, global_direction):
        total_start = time.perf_counter()
        p0 = np.asarray(p0, dtype='float64')
        if p0.shape != (3,):
            raise ValueError('p0 must have shape (3,)')
        direction = self._normalized_direction(global_direction)
        validation = self.collision_checker.validate(batch)
        valid_mask = (batch.feasible_mask & validation.fine_dynamic_mask &
                      validation.static_safe_mask)

        cost_start = time.perf_counter()
        progress = np.einsum('ij,j->i', batch.terminal_positions - p0, direction)
        progress_cost = -progress / config.PRIMITIVE_PROGRESS_SCALE
        lateral_scale = max(config.PRIMITIVE_LATERAL_SCALE, 1.0e-12)
        vertical_scale = max(config.PRIMITIVE_VERTICAL_SCALE, 1.0e-12)
        offset_cost = 0.5 * (np.abs(batch.lateral_offsets) / lateral_scale +
                             np.abs(batch.vertical_offsets) / vertical_scale)

        velocity = validation.velocities
        acceleration = validation.accelerations
        speed = np.linalg.norm(velocity, axis=2)
        cross = np.linalg.norm(np.cross(velocity, acceleration), axis=2)
        curvature = np.divide(cross, speed ** 3,
                              out=np.zeros_like(cross), where=speed > 1.0e-8)
        curvature_integrand = curvature ** 2 * speed
        curvature_integral = np.trapz(
            curvature_integrand, validation.validation_times, axis=1)
        curvature_cost = curvature_integral / config.PRIMITIVE_CURVATURE_REFERENCE

        clearance_deficit = np.maximum(
            config.PRIMITIVE_PREFERRED_CLEARANCE - validation.min_static_clearance,
            0.0)
        clearance_cost = (clearance_deficit / config.PRIMITIVE_CLEARANCE_SCALE) ** 2
        horizontal_ratio = (np.linalg.norm(acceleration[:, :, [0, 2]], axis=2) /
                            config.A_XZ_MAX)
        vertical_ratio = np.where(
            acceleration[:, :, 1] >= 0.0,
            acceleration[:, :, 1] / config.A_Y_UP_MAX,
            -acceleration[:, :, 1] / config.A_Y_DOWN_MAX)
        # Acceleration-use proxy only; this is NOT the official energy metric.
        effort_proxy = (np.trapz(
            horizontal_ratio ** 2 + vertical_ratio ** 2,
            validation.validation_times, axis=1) /
            self.collision_checker.horizon)
        total_cost = (
            config.PRIMITIVE_COST_WEIGHT_PROGRESS * progress_cost +
            config.PRIMITIVE_COST_WEIGHT_OFFSET * offset_cost +
            config.PRIMITIVE_COST_WEIGHT_CURVATURE * curvature_cost +
            config.PRIMITIVE_COST_WEIGHT_CLEARANCE * clearance_cost +
            config.PRIMITIVE_COST_WEIGHT_EFFORT_PROXY * effort_proxy)
        cost_time = time.perf_counter() - cost_start

        ranking_start = time.perf_counter()
        valid_indices = np.flatnonzero(valid_mask)
        if len(valid_indices):
            order = np.lexsort((valid_indices, total_cost[valid_indices]))
            ranked_indices = valid_indices[order]
            best_index = int(ranked_indices[0])
            success = True
            reason = 'success'
        else:
            ranked_indices = np.empty(0, dtype='int64')
            best_index = None
            success = False
            reason = 'no statically safe primitive'
        ranking_time = time.perf_counter() - ranking_start
        return StaticPrimitiveSelection(
            success, reason, batch, validation, validation.fine_dynamic_mask,
            validation.static_safe_mask, valid_mask,
            validation.min_static_clearance, progress, progress_cost,
            curvature_cost, clearance_cost, offset_cost, effort_proxy,
            total_cost, ranked_indices, best_index,
            validation.validation_times, validation.fine_validation_time,
            validation.static_collision_time, cost_time, ranking_time,
            time.perf_counter() - total_start)
