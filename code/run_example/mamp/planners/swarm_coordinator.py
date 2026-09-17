"""Atomic dynamic-priority coordination over existing single-UAV candidates."""

from dataclasses import dataclass
import time

import numpy as np

from ..collision.swarm_trajectory_collision import (audit_all_pairs,
    interval_minimum, SwarmTrajectoryCollisionChecker)
from ..configs import subject3_config as config


@dataclass
class TimeIndexedReservation:
    uav_id: int
    absolute_times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    selected_primitive_index: int


@dataclass
class SwarmCoordinationResult:
    success: bool
    reason: str
    planning_time: float
    priority_order: np.ndarray
    selected_indices: np.ndarray
    base_valid_counts: np.ndarray
    reservation_safe_counts: np.ndarray
    per_uav_min_pair_distance: np.ndarray
    reservations: tuple
    audit: object
    failure_snapshot: object
    base_single_uav_planning_total: float
    priority_build_time: float
    nominal_conflict_graph_time: float
    reservation_filter_time: float
    selection_time: float
    all_pairs_audit_time: float
    candidate_reservation_pair_checks: int


def initial_swarm_is_safe(positions, safe_distance=None):
    points = np.asarray(positions, dtype='float64')
    threshold = config.UAV_SAFE_DISTANCE if safe_distance is None else float(safe_distance)
    delta = points[:, None, :] - points[None, :, :]
    distance = np.linalg.norm(delta, axis=2)
    distance[np.diag_indices(len(points))] = np.inf
    return bool(np.min(distance) >= threshold - 1.e-12), float(np.min(distance))


def dynamic_priority(base_valid_counts, conflict_degrees,
                     environment_clearances, uav_ids):
    ids = np.asarray(uav_ids, dtype='int64')
    order = np.lexsort((ids, np.asarray(environment_clearances),
                        -np.asarray(conflict_degrees),
                        np.asarray(base_valid_counts)))
    return ids[order]


class SwarmCoordinator(object):
    def __init__(self, collision_checker=None, reservation_mode='first_safe',
                 candidate_chunk=4):
        self.collision_checker = collision_checker or SwarmTrajectoryCollisionChecker()
        if reservation_mode not in ('first_safe', 'full_debug'):
            raise ValueError('reservation_mode must be first_safe or full_debug')
        self.reservation_mode = reservation_mode
        self.candidate_chunk = int(candidate_chunk)

    @staticmethod
    def _trajectory(plan, candidate_index):
        validation = plan.static_selection.validation
        return (validation.validation_times,
                validation.positions[candidate_index],
                validation.velocities[candidate_index],
                validation.accelerations[candidate_index])

    def coordinate(self, plans, states, planning_time, uav_ids=None,
                   goal_versions=None):
        total_start = time.perf_counter()
        count = len(plans)
        ids = np.arange(count, dtype='int64') if uav_ids is None else np.asarray(uav_ids)
        positions = np.asarray([state[0] for state in states])
        valid_initial, initial_minimum = initial_swarm_is_safe(positions)
        base_total = float(sum(plan.planning_time for plan in plans))
        if not valid_initial:
            return SwarmCoordinationResult(
                False, 'invalid initial swarm state', time.perf_counter() - total_start,
                np.empty(0, dtype='int64'), np.full(count, -1),
                np.zeros(count, dtype='int64'), np.zeros(count, dtype='int64'),
                np.full(count, np.inf), tuple(), None,
                {'planning_time': float(planning_time),
                 'minimum_initial_pair_distance': initial_minimum},
                base_total, 0., 0., 0., 0., 0., 0)
        failed_base = [i for i, plan in enumerate(plans) if not plan.success]
        if failed_base:
            return SwarmCoordinationResult(
                False, 'base single-UAV planning failed', time.perf_counter() - total_start,
                np.empty(0, dtype='int64'), np.full(count, -1),
                np.zeros(count, dtype='int64'), np.zeros(count, dtype='int64'),
                np.full(count, np.inf), tuple(), None,
                {'failed_uav_id': int(ids[failed_base[0]]),
                 'base_failure': plans[failed_base[0]].failure_snapshot},
                base_total, 0., 0., 0., 0., 0., 0)

        graph_start = time.perf_counter()
        base_valid = np.asarray([len(plan.dynamic_selection.ranked_indices)
                                 for plan in plans], dtype='int64')
        best_positions = []
        environment_clearance = []
        for plan in plans:
            index = int(plan.dynamic_selection.ranked_indices[0])
            best_positions.append(self._trajectory(plan, index)[1])
            environment_clearance.append(min(
                plan.static_selection.min_static_clearance[index],
                plan.dynamic_selection.min_dynamic_clearance[index]))
        best_positions = np.asarray(best_positions)
        conflicts = np.zeros((count, count), dtype=bool)
        times = plans[0].static_selection.validation.validation_times
        pair_i, pair_j = np.triu_indices(count, k=1)
        pair_distance, _ = interval_minimum(
            best_positions[pair_i] - best_positions[pair_j], times)
        pair_conflict = pair_distance < (self.collision_checker.safe_distance +
                                         self.collision_checker.guard - 1.e-12)
        conflicts[pair_i, pair_j] = pair_conflict
        conflicts[pair_j, pair_i] = pair_conflict
        graph_time = time.perf_counter() - graph_start
        priority_start = time.perf_counter()
        degrees = np.sum(conflicts, axis=1)
        priority = dynamic_priority(base_valid, degrees, environment_clearance, ids)
        priority_time = time.perf_counter() - priority_start

        reservations, selected = [], np.full(count, -1, dtype='int64')
        safe_counts = np.zeros(count, dtype='int64')
        per_uav_minimum = np.full(count, np.inf)
        filter_time = selection_time = 0.0
        pair_checks = 0
        id_to_offset = {int(value): offset for offset, value in enumerate(ids)}
        for rank, uav_id in enumerate(priority):
            offset = id_to_offset[int(uav_id)]
            plan = plans[offset]
            ranked = plan.dynamic_selection.ranked_indices
            candidate_positions = plan.static_selection.validation.positions[ranked]
            check = None
            selected_rank = None
            if not reservations:
                selected_rank = 0
                safe_counts[offset] = len(ranked) if self.reservation_mode == 'full_debug' else -1
            elif self.reservation_mode == 'full_debug':
                filter_start = time.perf_counter()
                check = self.collision_checker.check_reservations(
                    candidate_positions, times, reservations)
                filter_time += time.perf_counter() - filter_start
                pair_checks += check.pair_checks
                safe_counts[offset] = int(np.count_nonzero(check.safe_mask))
                available = np.flatnonzero(check.safe_mask)
                selected_rank = None if not len(available) else int(available[0])
            else:
                stacked = np.asarray([item.positions for item in reservations])
                threshold = (self.collision_checker.safe_distance +
                             self.collision_checker.guard - 1.e-12)
                filter_start = time.perf_counter()
                for chunk_start in range(0, len(ranked), self.candidate_chunk):
                    chunk = candidate_positions[
                        chunk_start:chunk_start + self.candidate_chunk]
                    distance, _ = interval_minimum(
                        chunk[:, None, :, :] - stacked[None, :, :, :], times)
                    pair_checks += int(distance.size)
                    chunk_safe = np.all(distance >= threshold, axis=1)
                    available = np.flatnonzero(chunk_safe)
                    if len(available):
                        selected_rank = chunk_start + int(available[0])
                        per_uav_minimum[offset] = float(np.min(
                            distance[int(available[0])]))
                        break
                filter_time += time.perf_counter() - filter_start
                safe_counts[offset] = -1
            if selected_rank is None:
                if check is None:
                    check = self.collision_checker.check_reservations(
                        candidate_positions, times, reservations)
                versions = np.zeros(count, dtype='int64') if goal_versions is None else goal_versions
                closest_offset = int(np.argmin(check.min_pair_distance))
                failure = {
                    'planning_time': float(planning_time),
                    'priority_order': priority.copy(), 'failed_uav_id': int(uav_id),
                    'failed_priority_rank': rank,
                    'position': np.asarray(states[offset][0]).copy(),
                    'velocity': np.asarray(states[offset][1]).copy(),
                    'acceleration': np.asarray(states[offset][2]).copy(),
                    'goal_version': int(versions[offset]),
                    'guide_direction': plan.guide_direction.copy(),
                    'phase3_feasible': int(np.count_nonzero(plan.primitive_batch.feasible_mask)),
                    'phase4_valid': len(plan.static_selection.ranked_indices),
                    'phase5_valid': len(ranked),
                    'reservation_candidates_before': len(ranked),
                    'reservation_candidates_after': 0,
                    'eliminated_by_reservation': check.eliminated_by_reservation,
                    'closest_reserved_uav_id': int(check.closest_reserved_uav_id[closest_offset]),
                    'minimum_achievable_pair_distance': float(check.min_pair_distance[closest_offset]),
                    'reservations': tuple((r.uav_id, r.selected_primitive_index)
                                          for r in reservations)}
                return SwarmCoordinationResult(
                    False, 'no coordinated candidate', time.perf_counter() - total_start,
                    priority, selected, base_valid, safe_counts, per_uav_minimum,
                    tuple(reservations), None, failure, base_total, priority_time,
                    graph_time, filter_time, selection_time, 0., pair_checks)
            selection_start = time.perf_counter()
            original_index = int(ranked[selected_rank])
            if check is not None:
                per_uav_minimum[offset] = float(check.min_pair_distance[selected_rank])
            selection_time += time.perf_counter() - selection_start
            selected[offset] = original_index
            relative_times, p, v, a = self._trajectory(plan, original_index)
            reservations.append(TimeIndexedReservation(
                int(uav_id), float(planning_time) + relative_times,
                p.copy(), v.copy(), a.copy(), original_index))

        audit_start = time.perf_counter()
        audit = audit_all_pairs(reservations, self.collision_checker.safe_distance,
                                self.collision_checker.guard)
        audit_time = time.perf_counter() - audit_start
        success = audit.pair_violation_count == 0
        return SwarmCoordinationResult(
            success, 'success' if success else 'independent all-pairs audit failed',
            time.perf_counter() - total_start, priority, selected, base_valid,
            safe_counts, per_uav_minimum, tuple(reservations), audit, None,
            base_total, priority_time, graph_time, filter_time, selection_time,
            audit_time, pair_checks)
