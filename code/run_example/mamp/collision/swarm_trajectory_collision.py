"""Vectorized sampled/interpolated UAV-UAV trajectory separation checks."""

from dataclasses import dataclass
import math

import numpy as np

from ..configs import subject3_config as config


def interpolation_guard(dt=None):
    step = config.VALIDATION_DT if dt is None else float(dt)
    total_acceleration = math.sqrt(
        config.A_XZ_MAX ** 2 +
        max(config.A_Y_UP_MAX, config.A_Y_DOWN_MAX) ** 2)
    return 2.0 * total_acceleration * step * step / 8.0


def interval_minimum(relative_positions, times):
    """Return minimum linearly-interpolated relative distance and time."""
    relative = np.asarray(relative_positions, dtype='float64')
    times = np.asarray(times, dtype='float64')
    if relative.ndim < 2 or relative.shape[-1] != 3:
        raise ValueError('relative_positions must have shape (...,Nv,3)')
    leading_shape = relative.shape[:-2]
    squeeze = not leading_shape
    if relative.shape[-2] != len(times):
        raise ValueError('relative positions and times are inconsistent')
    relative = relative.reshape((-1, len(times), 3))
    r0, dr = relative[:, :-1, :], np.diff(relative, axis=1)
    denominator = np.einsum('cij,cij->ci', dr, dr)
    numerator = -np.einsum('cij,cij->ci', r0, dr)
    alpha = np.divide(numerator, denominator, out=np.zeros_like(numerator),
                      where=denominator > 1.0e-20)
    np.clip(alpha, 0.0, 1.0, out=alpha)
    closest = r0 + alpha[:, :, None] * dr
    distances = np.linalg.norm(closest, axis=2)
    indices = np.argmin(distances, axis=1)
    rows = np.arange(len(relative))
    minimum = distances[rows, indices]
    minimum_time = (times[indices] + alpha[rows, indices] *
                    (times[indices + 1] - times[indices]))
    if squeeze:
        return float(minimum[0]), float(minimum_time[0])
    return minimum.reshape(leading_shape), minimum_time.reshape(leading_shape)


@dataclass
class ReservationCheck:
    safe_mask: np.ndarray
    min_pair_distance: np.ndarray
    closest_reserved_uav_id: np.ndarray
    time_of_min_pair_distance: np.ndarray
    eliminated_by_reservation: dict
    pair_checks: int


class SwarmTrajectoryCollisionChecker(object):
    def __init__(self, safe_distance=None, guard=None):
        self.safe_distance = float(config.UAV_SAFE_DISTANCE if safe_distance is None
                                   else safe_distance)
        self.guard = float(interpolation_guard() if guard is None else guard)

    def check_reservations(self, candidate_positions, times, reservations):
        positions = np.asarray(candidate_positions, dtype='float64')
        count = len(positions)
        safe = np.ones(count, dtype=bool)
        minimum = np.full(count, np.inf)
        closest_id = np.full(count, -1, dtype='int64')
        minimum_time = np.full(count, np.nan)
        eliminated = {}
        threshold = self.safe_distance + self.guard
        for reservation in reservations:
            before = safe.copy()
            distance, when = interval_minimum(
                positions - reservation.positions[None, :, :], times)
            better = distance < minimum
            minimum[better] = distance[better]
            minimum_time[better] = when[better]
            closest_id[better] = reservation.uav_id
            safe &= distance >= threshold - 1.0e-12
            eliminated[reservation.uav_id] = int(np.count_nonzero(before & ~safe))
        return ReservationCheck(safe, minimum, closest_id, minimum_time,
                                eliminated, count * len(reservations))

    def pair_is_safe(self, first, second, times):
        distance, when = interval_minimum(first - second, times)
        return distance >= self.safe_distance + self.guard - 1.e-12, distance, when


@dataclass
class AllPairsAudit:
    global_min_pair_distance: float
    sampled_min_pair_distance: float
    closest_pair: tuple
    time_of_global_min_pair_distance: float
    pair_violation_count: int
    pair_count: int


def audit_all_pairs(reservations, safe_distance=None, guard=None):
    checker = SwarmTrajectoryCollisionChecker(safe_distance, guard)
    pair_count = len(reservations) * (len(reservations) - 1) // 2
    if not pair_count:
        return AllPairsAudit(float('inf'), float('inf'), None, float('nan'), 0, 0)
    positions = np.asarray([item.positions for item in reservations])
    ids = np.asarray([item.uav_id for item in reservations])
    pair_i, pair_j = np.triu_indices(len(reservations), k=1)
    relative = positions[pair_i] - positions[pair_j]
    distance, when = interval_minimum(relative, reservations[0].absolute_times)
    sampled = np.min(np.linalg.norm(relative, axis=2), axis=1)
    best = int(np.argmin(distance))
    violations = int(np.count_nonzero(
        distance < checker.safe_distance + checker.guard - 1.e-12))
    return AllPairsAudit(float(distance[best]), float(np.min(sampled)),
                         (int(ids[pair_i[best]]), int(ids[pair_j[best]])),
                         float(when[best]), violations, pair_count)
