"""Vectorized fine-horizon dynamic and static validation for primitive batches."""

from dataclasses import dataclass
import math
import time

import numpy as np

from ..configs import subject3_config as config
from ..planners.quintic_primitive import evaluate_quintics


@dataclass
class FineStaticValidation:
    validation_times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    fine_speed_ok: np.ndarray
    fine_altitude_ok: np.ndarray
    fine_xz_acc_ok: np.ndarray
    fine_y_up_ok: np.ndarray
    fine_y_down_ok: np.ndarray
    fine_dynamic_mask: np.ndarray
    min_static_clearance: np.ndarray
    static_safe_mask: np.ndarray
    fine_validation_time: float
    static_collision_time: float


class StaticPrimitiveCollisionChecker(object):
    """Check static geometry only; moving obstacles are explicitly excluded."""

    def __init__(self, obstacles, horizon=None, validation_dt=None,
                 collision_guard=None, constraint_epsilon=None):
        self.static_obstacles = tuple(obstacle for obstacle in obstacles
                                      if obstacle.is_static)
        self.horizon = float(config.PRIMITIVE_HORIZON if horizon is None else horizon)
        self.validation_dt = float(config.VALIDATION_DT if validation_dt is None
                                   else validation_dt)
        if self.horizon <= 0.0 or self.validation_dt <= 0.0:
            raise ValueError('horizon and validation_dt must be positive')
        intervals = int(math.ceil(self.horizon / self.validation_dt))
        self.validation_times = np.linspace(0.0, self.horizon, intervals + 1)
        configured_guard = config.STATIC_COLLISION_GUARD
        if collision_guard is None and configured_guard is None:
            collision_guard = 0.5 * config.V_MAX * self.validation_dt
        elif collision_guard is None:
            collision_guard = configured_guard
        self.collision_guard = float(collision_guard)
        if self.collision_guard < 0.0:
            raise ValueError('collision_guard must be nonnegative')
        self.constraint_epsilon = float(
            config.PRIMITIVE_CONSTRAINT_EPS if constraint_epsilon is None
            else constraint_epsilon)

    def validate(self, batch):
        validation_start = time.perf_counter()
        positions, velocities, accelerations = evaluate_quintics(
            batch.coefficients, self.validation_times)
        eps = self.constraint_epsilon
        speed = np.linalg.norm(velocities, axis=2)
        xz_acceleration = np.linalg.norm(accelerations[:, :, [0, 2]], axis=2)
        fine_speed_ok = np.max(speed, axis=1) <= config.V_MAX + eps
        fine_altitude_ok = np.all(
            (positions[:, :, 1] >= config.Y_MIN_ASSUMED - eps) &
            (positions[:, :, 1] <= config.Y_MAX + eps), axis=1)
        fine_xz_acc_ok = np.max(xz_acceleration, axis=1) <= config.A_XZ_MAX + eps
        fine_y_up_ok = np.max(accelerations[:, :, 1], axis=1) <= config.A_Y_UP_MAX + eps
        fine_y_down_ok = np.min(accelerations[:, :, 1], axis=1) >= -config.A_Y_DOWN_MAX - eps
        fine_dynamic_mask = (fine_speed_ok & fine_altitude_ok & fine_xz_acc_ok &
                             fine_y_up_ok & fine_y_down_ok)
        fine_validation_time = time.perf_counter() - validation_start

        collision_start = time.perf_counter()
        primitive_count, validation_count = positions.shape[:2]
        flat_points = positions.reshape(-1, 3)
        min_clearance = np.full(primitive_count, np.inf, dtype='float64')
        for obstacle in self.static_obstacles:
            distance = np.sqrt(obstacle.distance_sq_to_points(flat_points))
            obstacle_minimum = np.min(
                distance.reshape(primitive_count, validation_count), axis=1)
            np.minimum(min_clearance, obstacle_minimum, out=min_clearance)
        required = config.OBS_SAFE_DISTANCE + self.collision_guard
        static_safe_mask = min_clearance >= required - eps
        static_collision_time = time.perf_counter() - collision_start
        return FineStaticValidation(
            self.validation_times, positions, velocities, accelerations,
            fine_speed_ok, fine_altitude_ok, fine_xz_acc_ok, fine_y_up_ok,
            fine_y_down_ok, fine_dynamic_mask, min_clearance,
            static_safe_mask, fine_validation_time, static_collision_time)
