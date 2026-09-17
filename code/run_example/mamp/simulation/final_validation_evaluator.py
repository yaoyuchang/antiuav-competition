"""Frozen-system long-horizon evaluator and structured final report helpers."""

from dataclasses import dataclass
import time

import numpy as np

from ..configs import subject3_config as config
from ..planners.global_guide import GlobalGuideTracker
from ..planners.quintic_primitive import evaluate_quintics
from ..planners.recursive_feasibility_coordinator import RecursiveFeasibilityCoordinator
from ..planners.swarm_base_batch_planner import SwarmBaseBatchPlanner
from .swarm_long_horizon_evaluator import SwarmLongHorizonEvaluator
from .swarm_rolling import SynchronizedSwarmRollingOrchestrator


@dataclass
class FinalScenarioResult:
    label: str
    success: bool
    reason: str
    completed_cycles: int
    first_failure_cycle: object
    failed_uav_id: object
    mission_time: float
    cycle_logs: tuple
    failure_snapshot: object
    minimum_uav_distance: float
    minimum_static_clearance: float
    minimum_dynamic_clearance: float
    uav_violation_count: int
    static_violation_count: int
    dynamic_violation_count: int
    arrival_count: int
    goal_update_count: int
    global_guide_replan_count: int
    dynamic_rejection_count: int
    k3_solved_count: int
    k5_activation_count: int
    joint_search_count: int
    future_feasibility_rejection_count: int
    fallback_count: int
    timing_summary_ms: dict


def timing_summary(values):
    values = 1000. * np.asarray(values, dtype='float64')
    if not len(values):
        return dict((key, float('nan')) for key in
                    ('mean', 'p50', 'p95', 'p99', 'max'))
    return {'mean': float(np.mean(values)), 'p50': float(np.median(values)),
            'p95': float(np.percentile(values, 95.)),
            'p99': float(np.percentile(values, 99.)),
            'max': float(np.max(values))}


class FinalValidationEvaluator(object):
    def __init__(self, planners, obstacles, goal_managers=None,
                 global_guide_planner=None, cycle_deadline=.2,
                 arrival_handling='hold'):
        self.planners = tuple(planners)
        self.obstacles = tuple(obstacles)
        self.goal_managers = (None if goal_managers is None else
                              tuple(goal_managers))
        self.global_guide_planner = global_guide_planner
        self.cycle_deadline = float(cycle_deadline)
        self.batcher = SwarmBaseBatchPlanner()
        self.coordinator = RecursiveFeasibilityCoordinator(self.planners)
        self.safety = SwarmLongHorizonEvaluator(obstacles)
        self.execution_times = np.linspace(
            0., config.EXECUTION_HORIZON,
            int(np.ceil(config.EXECUTION_HORIZON /
                        config.EXECUTION_RECORD_DT)) + 1)
        self.arrival_handling = arrival_handling
        self.arrival_adapter = SynchronizedSwarmRollingOrchestrator(
            self.planners, obstacles, arrival_handling=arrival_handling)

    def run(self, label, starts, goals, cycles, stop_on_all_arrived=False):
        positions = np.asarray(starts, dtype='float64').copy()
        goals = np.asarray(goals, dtype='float64').copy()
        velocities = np.zeros_like(positions)
        accelerations = np.zeros_like(positions)
        arrival = np.zeros(len(positions), dtype=bool)
        logs = []
        minimum_uav = minimum_static = minimum_dynamic = np.inf
        uav_violations = static_violations = dynamic_violations = 0
        dynamic_rejections = goal_updates = guide_replans = 0
        failure = None
        mission_time = 0.
        for cycle in range(int(cycles)):
            goal_records = []
            if self.goal_managers is not None:
                for uav, manager in enumerate(self.goal_managers):
                    update = manager.update(positions[uav], mission_time)
                    goal_updates += 1
                    if update.switched:
                        if self.global_guide_planner is None:
                            failure = {'cycle': cycle, 'mission_time': mission_time,
                                       'reason': 'goal switch requires guide planner',
                                       'failed_uav_id': uav}
                            break
                        guide = self.global_guide_planner.plan(
                            positions[uav], update.base_goal)
                        if not guide.success:
                            failure = {'cycle': cycle, 'mission_time': mission_time,
                                       'reason': 'switched global guide failed',
                                       'failed_uav_id': uav,
                                       'guide_reason': guide.reason}
                            break
                        tracker = GlobalGuideTracker(
                            guide.waypoints, turn_aware_enabled=True,
                            turn_accel_ref_ratio=config.GUIDE_TURN_ACCEL_REF_RATIO)
                        self.planners[uav].replace_goal(
                            update.filtered_goal, tracker, reset_terminal=True)
                        guide_replans += 1
                    else:
                        self.planners[uav].replace_goal(update.filtered_goal)
                    goals[uav] = update.filtered_goal
                    goal_records.append(update)
                if failure is not None:
                    break
            states = tuple((positions[i], velocities[i], accelerations[i])
                           for i in range(len(positions)))
            cycle_start = time.perf_counter()
            if self.arrival_handling == 'hold':
                active_ids = np.flatnonzero(~arrival)
                plans = [None] * len(positions)
                if len(active_ids):
                    active_plans, _, base = self.batcher.plan(
                        [self.planners[i] for i in active_ids],
                        [states[i] for i in active_ids], mission_time)
                    for uav, plan in zip(active_ids, active_plans):
                        plans[int(uav)] = plan
                else:
                    base = self.arrival_adapter._zero_timing()
                for uav in np.flatnonzero(arrival):
                    plans[int(uav)] = self.arrival_adapter._arrived_plan(
                        self.planners[int(uav)], states[int(uav)], mission_time)
                plans = tuple(plans)
            else:
                plans, _, base = self.batcher.plan(
                    self.planners, states, mission_time)
            coordination = self.coordinator.coordinate(
                plans, states, mission_time)
            planning_elapsed = time.perf_counter() - cycle_start
            dynamic_rejections += sum(
                len(plan.static_selection.ranked_indices) -
                len(plan.dynamic_selection.ranked_indices) for plan in plans)
            if not coordination.success:
                raw = coordination.failure_snapshot
                failure = {'cycle': cycle, 'mission_time': mission_time,
                           'reason': coordination.reason,
                           'failed_uav_id': (None if raw is None else
                                             raw.get('failed_uav_id')),
                           'coordination_failure': raw,
                           'positions': positions.copy(),
                           'velocities': velocities.copy(),
                           'accelerations': accelerations.copy()}
                break
            if planning_elapsed >= self.cycle_deadline:
                failure = {'cycle': cycle, 'mission_time': mission_time,
                           'reason': 'complete swarm cycle deadline exceeded',
                           'failed_uav_id': None,
                           'complete_cycle_time': planning_elapsed}
                break
            segment_p, segment_v, segment_a = [], [], []
            for uav, index in enumerate(coordination.selected_indices):
                p, v, a = evaluate_quintics(
                    plans[uav].primitive_batch.coefficients[index:index + 1],
                    self.execution_times)
                segment_p.append(p[0]); segment_v.append(v[0]); segment_a.append(a[0])
            segment_p = np.asarray(segment_p)
            segment_v = np.asarray(segment_v)
            segment_a = np.asarray(segment_a)
            audit = self.safety.audit_segment(
                segment_p, mission_time + self.execution_times)
            minimum_uav = min(minimum_uav, audit.minimum_uav_distance)
            minimum_static = min(minimum_static, audit.minimum_static_clearance)
            minimum_dynamic = min(minimum_dynamic, audit.minimum_dynamic_clearance)
            uav_violations += audit.uav_violation_count
            static_violations += audit.static_violation_count
            dynamic_violations += audit.dynamic_violation_count
            resolution = getattr(coordination, 'local_joint_resolution', None)
            logs.append({
                'cycle': cycle, 'mission_time': mission_time,
                'positions': positions.copy(), 'velocities': velocities.copy(),
                'accelerations': accelerations.copy(),
                # Fine-grained (EXECUTION_RECORD_DT) samples for the whole
                # executed segment, kept for energy/smoothness integration
                # (mission_metrics.py) in addition to the cycle-boundary
                # snapshot above used by the rolling controller itself.
                'segment_positions': segment_p.copy(),
                'segment_velocities': segment_v.copy(),
                'segment_accelerations': segment_a.copy(),
                'segment_times': (mission_time + self.execution_times).copy(),
                'priority_order': coordination.priority_order.copy(),
                'conflict_cluster': (np.empty(0, dtype='int64') if resolution is None
                    else resolution.cluster_uav_ids.copy()),
                'joint_assignment': ({} if resolution is None else
                                     dict(resolution.selected_indices)),
                'k_level': getattr(coordination, 'recursive_k_level', None),
                'selected_indices': coordination.selected_indices.copy(),
                'minimum_pair_distance': audit.minimum_uav_distance,
                'minimum_static_clearance': audit.minimum_static_clearance,
                'minimum_dynamic_clearance': audit.minimum_dynamic_clearance,
                'phase3_time': base.phase3_generation,
                'phase4_time': (base.fine_validation + base.static_collision +
                                base.phase4_cost_ranking),
                'phase5_time': (base.dynamic_prediction + base.dynamic_collision +
                                base.phase5_cost_ranking),
                'coordination_time': coordination.planning_time,
                'total_cycle_time': planning_elapsed,
                'goal_updates': tuple(goal_records)})
            positions = segment_p[:, -1, :].copy()
            velocities = segment_v[:, -1, :].copy()
            accelerations = segment_a[:, -1, :].copy()
            mission_time += config.EXECUTION_HORIZON
            arrival |= np.linalg.norm(positions - goals, axis=1) <= \
                config.GOAL_TOLERANCE_ASSUMED
            if self.arrival_handling == 'hold' and np.any(arrival):
                velocities[arrival] = 0.
                accelerations[arrival] = 0.
            if (audit.uav_violation_count or audit.static_violation_count or
                    audit.dynamic_violation_count):
                failure = {'cycle': cycle, 'mission_time': mission_time,
                           'reason': 'executed segment safety audit failed',
                           'failed_uav_id': None, 'audit': audit}
                break
            if stop_on_all_arrived and np.all(arrival):
                break
        success = failure is None and (len(logs) == int(cycles) or
                                       (stop_on_all_arrived and np.all(arrival)))
        return FinalScenarioResult(
            label, success, 'success' if success else failure['reason'],
            len(logs), None if failure is None else failure['cycle'],
            None if failure is None else failure.get('failed_uav_id'),
            mission_time, tuple(logs), failure, minimum_uav, minimum_static,
            minimum_dynamic, uav_violations, static_violations,
            dynamic_violations, int(np.count_nonzero(arrival)), goal_updates,
            guide_replans, dynamic_rejections,
            self.coordinator.k3_solved_count,
            self.coordinator.k5_activation_count,
            self.coordinator.joint_search_count,
            self.coordinator.future_feasibility_rejection_count,
            self.coordinator.fallback_count,
            timing_summary([item['total_cycle_time'] for item in logs]))


def report_dict(results):
    return dict((item.label, {
        'success': item.success, 'cycles': item.completed_cycles,
        'first_failure_cycle': item.first_failure_cycle,
        'failed_uav_id': item.failed_uav_id,
        'timing_ms': item.timing_summary_ms,
        'minimum_uav_distance': item.minimum_uav_distance,
        'minimum_static_clearance': item.minimum_static_clearance,
        'minimum_dynamic_clearance': item.minimum_dynamic_clearance,
        'arrival_count': item.arrival_count,
        'goal_update_count': item.goal_update_count,
        'guide_replan_count': item.global_guide_replan_count,
        'dynamic_rejection_count': item.dynamic_rejection_count})
                for item in results)
