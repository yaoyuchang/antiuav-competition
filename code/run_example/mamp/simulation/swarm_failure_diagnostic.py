"""Read-only diagnostics for Phase-6B reservation failures."""

from dataclasses import dataclass

import numpy as np

from ..collision.swarm_trajectory_collision import interval_minimum


@dataclass
class ReservationFailureDiagnostic:
    failed_uav_id: int
    ranked_candidate_indices: np.ndarray
    blocking_uav_ids: np.ndarray
    rejection_matrix: np.ndarray
    pair_minimum_distances: np.ndarray
    blocker_contribution_counts: np.ndarray
    blocker_unique_counts: np.ndarray
    candidate_blocker_counts: np.ndarray
    oracle_priority: np.ndarray
    oracle_success: bool
    oracle_failed_uav_id: object
    oracle_selected_indices: np.ndarray
    failed_uav_safe_when_first: int
    arrived_uav_count: int
    arrived_uav_ids: np.ndarray
    classification: str


def _trajectory(plan, candidate_index):
    validation = plan.static_selection.validation
    return validation.positions[candidate_index]


def _oracle_coordinate(plans, states, priority, checker):
    """Diagnostic-only greedy replay under a supplied priority order."""
    selected = np.full(len(plans), -1, dtype='int64')
    reserved_ids, reserved_positions = [], []
    times = plans[0].static_selection.validation.validation_times
    threshold = checker.safe_distance + checker.guard - 1.e-12
    for uav_id in priority:
        ranked = plans[uav_id].dynamic_selection.ranked_indices
        candidates = plans[uav_id].static_selection.validation.positions[ranked]
        if not reserved_positions:
            safe = np.ones(len(ranked), dtype=bool)
        else:
            stacked = np.asarray(reserved_positions)
            distance, _ = interval_minimum(
                candidates[:, None, :, :] - stacked[None, :, :, :], times)
            safe = np.all(distance >= threshold, axis=1)
        available = np.flatnonzero(safe)
        if not len(available):
            return False, int(uav_id), selected
        index = int(ranked[int(available[0])])
        selected[uav_id] = index
        reserved_ids.append(int(uav_id))
        reserved_positions.append(_trajectory(plans[uav_id], index))
    return True, None, selected


def diagnose_reservation_failure(plans, states, coordination, collision_checker,
                                 arrived_mask=None):
    """Explain one ``no coordinated candidate`` result without mutating it."""
    failure = coordination.failure_snapshot
    failed = int(failure['failed_uav_id'])
    ranked = plans[failed].dynamic_selection.ranked_indices.copy()
    reservations = coordination.reservations
    blockers = np.asarray([x.uav_id for x in reservations], dtype='int64')
    times = plans[failed].static_selection.validation.validation_times
    candidates = plans[failed].static_selection.validation.positions[ranked]
    if len(reservations):
        stacked = np.asarray([x.positions for x in reservations])
        distance, _ = interval_minimum(
            candidates[:, None, :, :] - stacked[None, :, :, :], times)
    else:
        distance = np.empty((len(ranked), 0))
    rejection = distance < (collision_checker.safe_distance +
                            collision_checker.guard - 1.e-12)
    contribution = np.sum(rejection, axis=0, dtype='int64')
    blocker_count = np.sum(rejection, axis=1, dtype='int64')
    unique = np.asarray([np.count_nonzero(
        rejection[:, column] & (blocker_count == 1))
        for column in range(len(blockers))], dtype='int64')
    original = coordination.priority_order
    oracle_priority = np.r_[failed, original[original != failed]]
    oracle_success, oracle_failed, oracle_selected = _oracle_coordinate(
        plans, states, oracle_priority, collision_checker)
    arrived = (np.zeros(len(plans), dtype=bool) if arrived_mask is None else
               np.asarray(arrived_mask, dtype=bool))
    arrived_ids = np.flatnonzero(arrived)
    arrived_blocking = np.isin(blockers, arrived_ids)
    without_arrived_safe = bool(len(ranked) and np.any(
        ~np.any(rejection[:, ~arrived_blocking], axis=1)))
    if oracle_success:
        classification = 'A. priority starvation'
    elif np.any(arrived_blocking) and without_arrived_safe:
        classification = 'C. arrived UAV reservation blocking'
    elif not len(ranked) or not oracle_success:
        classification = 'B. candidate family insufficient'
    else:
        classification = 'D. true geometric deadlock'
    return ReservationFailureDiagnostic(
        failed, ranked, blockers, rejection, distance, contribution, unique,
        blocker_count, oracle_priority, oracle_success, oracle_failed,
        oracle_selected, len(ranked), int(np.count_nonzero(arrived)),
        arrived_ids, classification)
