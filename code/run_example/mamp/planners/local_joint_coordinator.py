"""Bounded local joint assignment for failed sequential reservations."""

from dataclasses import dataclass
import time

import numpy as np

from ..collision.swarm_trajectory_collision import audit_all_pairs, interval_minimum
from .swarm_coordinator import (SwarmCoordinator, SwarmCoordinationResult,
                                TimeIndexedReservation)


@dataclass
class LocalJointResolution:
    attempted: bool
    solved: bool
    cluster_uav_ids: np.ndarray
    candidate_node_count: int
    conflict_edge_count: int
    enumeration_count: int
    enumeration_upper_bound: int
    selected_indices: dict
    rank_cost: object
    minimum_pair_distance: object
    fallback_reason: object
    planning_time: float


def candidate_blocking_matrix(plan, reservations, checker):
    ranked = plan.dynamic_selection.ranked_indices
    positions = plan.static_selection.validation.positions[ranked]
    times = plan.static_selection.validation.validation_times
    if not reservations:
        return np.zeros((len(ranked), 0), dtype=bool)
    reserved = np.asarray([item.positions for item in reservations])
    distance, _ = interval_minimum(
        positions[:, None, :, :] - reserved[None, :, :, :], times)
    return distance < checker.safe_distance + checker.guard - 1.e-12


def extract_failure_cluster(plan, reservations, checker, failed_uav_id,
                            maximum_size=5):
    rejection = candidate_blocking_matrix(plan, reservations, checker)
    contributions = np.sum(rejection, axis=0, dtype='int64')
    reservation_ids = np.asarray([item.uav_id for item in reservations],
                                 dtype='int64')
    order = np.lexsort((reservation_ids, -contributions))
    blockers = [int(reservation_ids[index]) for index in order
                if contributions[index] > 0]
    members = [int(failed_uav_id)] + blockers[:max(int(maximum_size) - 1, 0)]
    return np.asarray(members, dtype='int64'), rejection, contributions


def _safe_options(plan, fixed_reservations, checker, top_k):
    ranked = plan.dynamic_selection.ranked_indices
    positions = plan.static_selection.validation.positions[ranked]
    times = plan.static_selection.validation.validation_times
    if fixed_reservations:
        fixed = np.asarray([item.positions for item in fixed_reservations])
        threshold = checker.safe_distance + checker.guard - 1.e-12
        safe_ranks = []
        for start in range(0, len(ranked), 4):
            chunk = positions[start:start + 4]
            distance, _ = interval_minimum(
                chunk[:, None, :, :] - fixed[None, :, :, :], times)
            available = np.flatnonzero(np.all(distance >= threshold, axis=1))
            safe_ranks.extend((start + available).tolist())
            if len(safe_ranks) >= int(top_k):
                break
        ranks = np.asarray(safe_ranks[:int(top_k)], dtype='int64')
    else:
        ranks = np.arange(min(len(ranked), int(top_k)), dtype='int64')
    return tuple((int(rank), int(ranked[rank]), positions[rank]) for rank in ranks)


def solve_local_joint_assignment(plans, cluster_uav_ids, fixed_reservations,
                                 checker, top_k=3, maximum_combinations=250000):
    """Enumerate at most K^N local assignments using sum of formal ranks."""
    start = time.perf_counter()
    cluster = np.asarray(cluster_uav_ids, dtype='int64')
    if len(cluster) > 8:
        return LocalJointResolution(True, False, cluster, 0, 0, 0, 0, {},
                                    None, None, 'cluster size exceeds 8',
                                    time.perf_counter() - start)
    bounded_k = int(top_k)
    options = [_safe_options(plans[uav], fixed_reservations, checker, bounded_k)
               for uav in cluster]
    node_count = sum(len(item) for item in options)
    upper_bound = int(np.prod([len(item) for item in options], dtype='int64'))
    if any(not len(item) for item in options):
        return LocalJointResolution(True, False, cluster, node_count, 0, 0,
                                    upper_bound, {}, None, None,
                                    'a cluster UAV has no fixed-safe top-K candidate',
                                    time.perf_counter() - start)
    times = plans[int(cluster[0])].static_selection.validation.validation_times
    threshold = checker.safe_distance + checker.guard - 1.e-12
    conflict_edges = 0
    pair_distances = {}
    for first in range(len(cluster)):
        for second in range(first + 1, len(cluster)):
            first_positions = np.asarray([item[2] for item in options[first]])
            second_positions = np.asarray([item[2] for item in options[second]])
            distance, _ = interval_minimum(
                first_positions[:, None, :, :] -
                second_positions[None, :, :, :], times)
            pair_distances[(first, second)] = distance
            conflict_edges += int(np.count_nonzero(distance < threshold))
    best = None
    enumerated = 0
    selected_options = []
    def search(depth, running_minimum):
        nonlocal best, enumerated
        if enumerated >= int(maximum_combinations):
            return
        if depth == len(cluster):
            enumerated += 1
            combination = tuple(options[index][option_index]
                                for index, option_index in enumerate(selected_options))
            minimum_distance = running_minimum
            rank_cost = int(sum(item[0] for item in combination))
            indices = tuple(item[1] for item in combination)
            key = (-minimum_distance, rank_cost, indices)
            if best is None or key < best[0]:
                best = (key, combination, minimum_distance)
            return
        for option_index in range(len(options[depth])):
            distances = [pair_distances[(previous, depth)][
                selected_options[previous], option_index]
                for previous in range(depth)]
            enumerated += 1
            if any(value < threshold for value in distances):
                if enumerated >= int(maximum_combinations):
                    return
                continue
            selected_options.append(option_index)
            candidate_minimum = (running_minimum if not distances else
                min(running_minimum, float(np.min(distances))))
            search(depth + 1, candidate_minimum)
            selected_options.pop()
            if enumerated >= int(maximum_combinations):
                return
    search(0, float('inf'))
    if best is None:
        return LocalJointResolution(True, False, cluster, node_count,
                                    conflict_edges, enumerated, upper_bound, {},
                                    None, None, 'no safe top-K joint assignment',
                                    time.perf_counter() - start)
    selected = dict((int(uav), int(item[1]))
                    for uav, item in zip(cluster, best[1]))
    return LocalJointResolution(True, True, cluster, node_count,
                                conflict_edges, enumerated, upper_bound, selected,
                                best[0][1], best[2], None,
                                time.perf_counter() - start)


class LocalJointCoordinator(SwarmCoordinator):
    def __init__(self, *args, **kwargs):
        self.top_k = int(kwargs.pop('top_k', 3))
        self.maximum_cluster_size = int(kwargs.pop('maximum_cluster_size', 5))
        self.expand_priority_followers = bool(
            kwargs.pop('expand_priority_followers', False))
        super(LocalJointCoordinator, self).__init__(*args, **kwargs)
        self.joint_solved_count = 0
        self.fallback_count = 0

    def coordinate(self, plans, states, planning_time, uav_ids=None,
                   goal_versions=None):
        total_start = time.perf_counter()
        baseline = super(LocalJointCoordinator, self).coordinate(
            plans, states, planning_time, uav_ids, goal_versions)
        empty = LocalJointResolution(False, False, np.empty(0, dtype='int64'),
                                     0, 0, 0, 0, {}, None, None, None, 0.)
        if baseline.success or baseline.reason != 'no coordinated candidate':
            baseline.local_joint_resolution = empty
            baseline.joint_solved_count = self.joint_solved_count
            baseline.fallback_count = self.fallback_count
            baseline.local_joint_overhead = 0.
            return baseline
        failed = int(baseline.failure_snapshot['failed_uav_id'])
        cluster, _, _ = extract_failure_cluster(
            plans[failed], baseline.reservations, self.collision_checker, failed,
            self.maximum_cluster_size)
        if self.expand_priority_followers:
            # Terminal congestion involves both already-reserved and pending
            # neighbours.  Expand by current spatial proximity so the whole
            # dense destination group can be reassigned together.
            expanded = [int(value) for value in cluster]
            ids = (np.arange(len(states), dtype='int64') if uav_ids is None
                   else np.asarray(uav_ids, dtype='int64'))
            id_to_offset = {int(value): offset for offset, value in enumerate(ids)}
            failed_position = np.asarray(states[id_to_offset[failed]][0])
            proximity = []
            for candidate_id in ids:
                candidate = int(candidate_id)
                distance = float(np.linalg.norm(
                    np.asarray(states[id_to_offset[candidate]][0]) - failed_position))
                proximity.append((distance, candidate))
            for _, candidate in sorted(proximity):
                if candidate not in expanded:
                    expanded.append(candidate)
                if len(expanded) >= self.maximum_cluster_size:
                    break
            cluster = np.asarray(expanded, dtype='int64')
        cluster_set = set(int(x) for x in cluster)
        fixed = [item for item in baseline.reservations
                 if item.uav_id not in cluster_set]
        resolution = solve_local_joint_assignment(
            plans, cluster, fixed, self.collision_checker, self.top_k)
        if not resolution.solved:
            self.fallback_count += 1
            baseline.local_joint_resolution = resolution
            baseline.joint_solved_count = self.joint_solved_count
            baseline.fallback_count = self.fallback_count
            baseline.local_joint_overhead = time.perf_counter() - total_start - baseline.planning_time
            return baseline

        ids = np.arange(len(plans), dtype='int64') if uav_ids is None else np.asarray(uav_ids)
        id_to_offset = dict((int(value), offset) for offset, value in enumerate(ids))
        selected = baseline.selected_indices.copy()
        reservations = list(fixed)
        relative_times = plans[0].static_selection.validation.validation_times
        for uav_id in baseline.priority_order:
            uav = int(uav_id)
            if uav not in cluster_set:
                continue
            offset = id_to_offset[uav]
            index = resolution.selected_indices[uav]
            selected[offset] = index
            _, p, v, a = self._trajectory(plans[offset], index)
            reservations.append(TimeIndexedReservation(
                uav, float(planning_time) + relative_times,
                p.copy(), v.copy(), a.copy(), index))

        already = set(item.uav_id for item in reservations)
        threshold = self.collision_checker.safe_distance + self.collision_checker.guard - 1.e-12
        extra_checks = 0
        for uav_id in baseline.priority_order:
            uav = int(uav_id)
            if uav in already:
                continue
            offset = id_to_offset[uav]
            ranked = plans[offset].dynamic_selection.ranked_indices
            candidate_positions = plans[offset].static_selection.validation.positions[ranked]
            stacked = np.asarray([item.positions for item in reservations])
            chosen_rank = None
            for chunk_start in range(0, len(ranked), self.candidate_chunk):
                chunk = candidate_positions[chunk_start:chunk_start + self.candidate_chunk]
                distance, _ = interval_minimum(
                    chunk[:, None, :, :] - stacked[None, :, :, :], relative_times)
                extra_checks += int(distance.size)
                available = np.flatnonzero(np.all(distance >= threshold, axis=1))
                if len(available):
                    chosen_rank = chunk_start + int(available[0])
                    break
            if chosen_rank is None:
                self.fallback_count += 1
                resolution.solved = False
                resolution.fallback_reason = ('post-joint sequential continuation '
                                              'failed at UAV{}'.format(uav))
                baseline.local_joint_resolution = resolution
                baseline.joint_solved_count = self.joint_solved_count
                baseline.fallback_count = self.fallback_count
                baseline.local_joint_overhead = time.perf_counter() - total_start - baseline.planning_time
                return baseline
            index = int(ranked[chosen_rank])
            selected[offset] = index
            _, p, v, a = self._trajectory(plans[offset], index)
            reservations.append(TimeIndexedReservation(
                uav, float(planning_time) + relative_times,
                p.copy(), v.copy(), a.copy(), index))
            already.add(uav)

        audit = audit_all_pairs(reservations, self.collision_checker.safe_distance,
                                self.collision_checker.guard)
        if audit.pair_violation_count:
            self.fallback_count += 1
            resolution.solved = False
            resolution.fallback_reason = 'post-joint independent audit failed'
            baseline.local_joint_resolution = resolution
            baseline.joint_solved_count = self.joint_solved_count
            baseline.fallback_count = self.fallback_count
            baseline.local_joint_overhead = time.perf_counter() - total_start - baseline.planning_time
            return baseline
        self.joint_solved_count += 1
        elapsed = time.perf_counter() - total_start
        result = SwarmCoordinationResult(
            True, 'success after local joint assignment', elapsed,
            baseline.priority_order.copy(), selected,
            np.asarray([len(x.dynamic_selection.ranked_indices) for x in plans]),
            np.full(len(plans), -1, dtype='int64'), np.full(len(plans), np.inf),
            tuple(reservations), audit, None,
            baseline.base_single_uav_planning_total, baseline.priority_build_time,
            baseline.nominal_conflict_graph_time, baseline.reservation_filter_time,
            baseline.selection_time, baseline.all_pairs_audit_time,
            baseline.candidate_reservation_pair_checks + extra_checks)
        result.local_joint_resolution = resolution
        result.joint_solved_count = self.joint_solved_count
        result.fallback_count = self.fallback_count
        result.local_joint_overhead = elapsed - baseline.planning_time
        return result
