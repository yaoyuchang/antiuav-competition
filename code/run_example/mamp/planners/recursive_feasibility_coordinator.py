"""Adaptive local joint coordination with a one-step future feasibility gate."""

import time

from .local_joint_coordinator import LocalJointCoordinator
from .swarm_coordinator import SwarmCoordinator
from ..simulation.swarm_shadow_rollout import one_step_future_feasibility


class RecursiveFeasibilityCoordinator(SwarmCoordinator):
    def __init__(self, planners, future_feasibility_enabled=True, *args, **kwargs):
        super(RecursiveFeasibilityCoordinator, self).__init__(*args, **kwargs)
        self.planners = tuple(planners)
        self.future_feasibility_enabled = bool(future_feasibility_enabled)
        self.k3_solved_count = 0
        self.k5_activation_count = 0
        self.joint_search_count = 0
        self.future_feasibility_rejection_count = 0
        self.fallback_count = 0

    def _decorate(self, result, k_level=None, future=None, overhead=0.):
        result.recursive_k_level = k_level
        result.shadow_future_feasibility = future
        result.recursive_coordination_overhead = float(overhead)
        result.k3_solved_count = self.k3_solved_count
        result.k5_activation_count = self.k5_activation_count
        result.joint_search_count = self.joint_search_count
        result.future_feasibility_rejection_count = (
            self.future_feasibility_rejection_count)
        result.fallback_count = self.fallback_count
        return result

    def coordinate(self, plans, states, planning_time, uav_ids=None,
                   goal_versions=None):
        start = time.perf_counter()
        k3 = LocalJointCoordinator(
            collision_checker=self.collision_checker,
            reservation_mode=self.reservation_mode,
            candidate_chunk=self.candidate_chunk, top_k=3)
        first = k3.coordinate(plans, states, planning_time, uav_ids,
                              goal_versions)
        resolution = first.local_joint_resolution
        if not resolution.attempted:
            return self._decorate(first, overhead=time.perf_counter() - start -
                                  first.planning_time)
        self.joint_search_count += 1
        if first.success:
            future = None
            if self.future_feasibility_enabled:
                future = one_step_future_feasibility(
                    self.planners, plans, first.selected_indices, planning_time,
                    uav_ids=resolution.cluster_uav_ids)
            if not self.future_feasibility_enabled or future.success:
                self.k3_solved_count += 1
                return self._decorate(
                    first, 3, future,
                    time.perf_counter() - start - first.planning_time)
            self.future_feasibility_rejection_count += 1

        self.k5_activation_count += 1
        self.joint_search_count += 1
        k5 = LocalJointCoordinator(
            collision_checker=self.collision_checker,
            reservation_mode=self.reservation_mode,
            candidate_chunk=self.candidate_chunk, top_k=30)
        second = k5.coordinate(plans, states, planning_time, uav_ids,
                               goal_versions)
        future = None
        if second.success and self.future_feasibility_enabled:
            future = one_step_future_feasibility(
                self.planners, plans, second.selected_indices, planning_time,
                uav_ids=second.local_joint_resolution.cluster_uav_ids)
        if second.success and (not self.future_feasibility_enabled or future.success):
            return self._decorate(
                second, 5, future,
                time.perf_counter() - start - second.planning_time)
        if second.success:
            self.future_feasibility_rejection_count += 1
        resolution = second.local_joint_resolution
        if (not second.success and resolution.fallback_reason is not None and
                resolution.fallback_reason.startswith(
                    'post-joint sequential continuation failed')):
            expanded = LocalJointCoordinator(
                collision_checker=self.collision_checker,
                reservation_mode=self.reservation_mode,
                candidate_chunk=self.candidate_chunk, top_k=30,
                maximum_cluster_size=8,
                expand_priority_followers=True).coordinate(
                    plans, states, planning_time, uav_ids, goal_versions)
            expanded_future = None
            if expanded.success and self.future_feasibility_enabled:
                expanded_future = one_step_future_feasibility(
                    self.planners, plans, expanded.selected_indices,
                    planning_time,
                    uav_ids=expanded.local_joint_resolution.cluster_uav_ids)
            if (expanded.success and (not self.future_feasibility_enabled or
                                      expanded_future.success)):
                return self._decorate(
                    expanded, 30, expanded_future,
                    time.perf_counter() - start - expanded.planning_time)
            if expanded.success:
                self.future_feasibility_rejection_count += 1
        self.fallback_count += 1
        fallback = super(RecursiveFeasibilityCoordinator, self).coordinate(
            plans, states, planning_time, uav_ids, goal_versions)
        if fallback.failure_snapshot is not None:
            fallback.failure_snapshot['joint_repair_diagnostics'] = {
                'k3': {
                    'cluster': first.local_joint_resolution.cluster_uav_ids.copy(),
                    'reason': first.local_joint_resolution.fallback_reason,
                    'nodes': first.local_joint_resolution.candidate_node_count,
                    'combinations': first.local_joint_resolution.enumeration_count,
                },
                'expanded': {
                    'cluster': second.local_joint_resolution.cluster_uav_ids.copy(),
                    'reason': second.local_joint_resolution.fallback_reason,
                    'nodes': second.local_joint_resolution.candidate_node_count,
                    'combinations': second.local_joint_resolution.enumeration_count,
                },
            }
        return self._decorate(
            fallback, 5, future,
            time.perf_counter() - start - fallback.planning_time)
