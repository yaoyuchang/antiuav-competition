"""Synchronized 24-UAV receding-horizon execution over the Phase-6A.5 path."""

from dataclasses import dataclass
import time

import numpy as np

from ..configs import subject3_config as config
from ..planners.quintic_primitive import evaluate_quintics
from ..planners.quintic_primitive import PrimitiveBatch
from ..planners.swarm_base_batch_planner import (BatchBasePlan,
    BatchPlanningTiming, SwarmBaseBatchPlanner)
from ..planners.swarm_coordinator import SwarmCoordinator
from .swarm_long_horizon_evaluator import SwarmLongHorizonEvaluator
from .swarm_failure_diagnostic import diagnose_reservation_failure


@dataclass
class SwarmRollingResult:
    success: bool
    reason: str
    completed_cycles: int
    mission_time: float
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    cycle_records: tuple
    failure_snapshot: object
    minimum_uav_distance: float
    minimum_static_clearance: float
    minimum_dynamic_clearance: float
    uav_violation_count: int
    static_violation_count: int
    dynamic_violation_count: int
    priority_change_count: int
    priority_change_cycles: int
    selected_index_counts: tuple
    zero_base_candidate_events: int
    minimum_base_candidate_count: int
    arrival_cycles: np.ndarray
    terminal_statistics: object = None
    arrival_handling: str = 'hold'
    overshoot_count: int = 0


class SynchronizedSwarmRollingOrchestrator(object):
    """Plan atomically, then execute the same 0.2 s prefix for every UAV.

    The legacy ``continue`` mode is retained for exact replay.  F2 ``hold``
    mode represents an arrived UAV by one stationary reservation trajectory;
    it remains in coordination, obstacle checks, history, and safety audits.
    """

    def __init__(self, planners, obstacles, execution_horizon=None,
                 record_dt=None, cycle_deadline=0.2, batch_planner=None,
                 coordinator=None, evaluator=None, arrival_handling='hold'):
        self.planners = tuple(planners)
        self.obstacles = tuple(obstacles)
        self.execution_horizon = float(
            config.EXECUTION_HORIZON if execution_horizon is None else execution_horizon)
        self.record_dt = float(
            config.EXECUTION_RECORD_DT if record_dt is None else record_dt)
        self.cycle_deadline = float(cycle_deadline)
        self.batch_planner = batch_planner or SwarmBaseBatchPlanner()
        self.coordinator = coordinator or SwarmCoordinator()
        self.evaluator = evaluator or SwarmLongHorizonEvaluator(obstacles)
        if arrival_handling not in ('continue', 'hold'):
            raise ValueError('arrival_handling must be continue or hold')
        self.arrival_handling = arrival_handling
        intervals = int(np.ceil(self.execution_horizon / self.record_dt))
        self.execution_times = np.linspace(0., self.execution_horizon, intervals + 1)

    @staticmethod
    def terminal_feasibility(position, velocity, goal):
        """State-only F2 monitor; it has no influence on candidate selection."""
        delta = np.asarray(goal) - np.asarray(position)
        distance = float(np.linalg.norm(delta))
        direction = (np.zeros(3) if distance <= 1.e-12 else delta / distance)
        forward = float(np.dot(velocity, direction))
        required = max((forward ** 2 - config.TERMINAL_CAPTURE_SPEED ** 2) /
                       (2. * config.TERMINAL_DECEL_REF), 0.)
        rho = required / max(distance, 1.e-12)
        return {'distance_to_goal': distance,
                'speed': float(np.linalg.norm(velocity)),
                'forward_goal_speed': forward,
                'required_capture_distance': required, 'rho': rho}

    @staticmethod
    def _zero_timing():
        return BatchPlanningTiming(0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                   0, 0, 0, 0, 0, 0)

    def _arrived_plan(self, planner, state, mission_time):
        """Build the safety-checked stationary reservation for an ARRIVED UAV."""
        position = np.asarray(state[0], dtype='float64')
        times = planner.generator.times
        count = len(times)
        positions = np.broadcast_to(position, (1, count, 3)).copy()
        velocities = np.zeros_like(positions)
        accelerations = np.zeros_like(positions)
        coefficients = np.zeros((1, 6, 3), dtype='float64')
        coefficients[0, 0] = position
        true = np.ones(1, dtype=bool)
        zeros = np.zeros(1, dtype='float64')
        batch = PrimitiveBatch(
            times.copy(), positions, velocities, accelerations, coefficients,
            position.reshape(1, 3).copy(), np.zeros((1, 3)), np.zeros((1, 3)),
            zeros.copy(), zeros.copy(), zeros.copy(), true.copy(), true.copy(),
            true.copy(), true.copy(), true.copy(), true.copy(), zeros.copy(),
            zeros.copy(), zeros.copy(), zeros.copy())
        delta = planner.goal - position
        norm = float(np.linalg.norm(delta))
        direction = delta / norm if norm > 1.e-12 else np.array([1., 0., 0.])
        static = planner.static_selector.select(batch, position, direction)
        dynamic = planner.dynamic_selector.select(static, mission_time)
        success = static.success and dynamic.success
        return BatchBasePlan(success,
            'success' if success else 'arrived stationary reservation unsafe',
            0., batch, static, dynamic, direction, planner.goal.copy(), 'ARRIVED')

    @staticmethod
    def _snapshot(cycle, mission_time, reason, states, timing=None,
                  coordination=None, audit=None):
        return {
            'cycle': int(cycle), 'mission_time': float(mission_time),
            'reason': reason,
            'positions': np.asarray([x[0] for x in states]).copy(),
            'velocities': np.asarray([x[1] for x in states]).copy(),
            'accelerations': np.asarray([x[2] for x in states]).copy(),
            'base_timing': timing,
            'coordination_failure': (None if coordination is None else
                                     coordination.failure_snapshot),
            'priority_order': (None if coordination is None else
                               coordination.priority_order.copy()),
            'selected_indices': (None if coordination is None else
                                 coordination.selected_indices.copy()),
            'executed_segment_audit': audit,
        }

    def run(self, starts, goals, velocities=None, accelerations=None,
            cycles=500, mission_time=0.):
        starts = np.asarray(starts, dtype='float64')
        goals = np.asarray(goals, dtype='float64')
        count = len(starts)
        if len(self.planners) != count or goals.shape != starts.shape:
            raise ValueError('planner/start/goal counts must agree')
        velocities = (np.zeros_like(starts) if velocities is None else
                      np.asarray(velocities, dtype='float64').copy())
        accelerations = (np.zeros_like(starts) if accelerations is None else
                         np.asarray(accelerations, dtype='float64').copy())
        positions = starts.copy()
        states = [(positions[i], velocities[i], accelerations[i])
                  for i in range(count)]
        records, selected_counts = [], [dict() for _ in range(count)]
        arrival_cycles = np.full(count, -1, dtype='int64')
        arrival_positions = np.full_like(positions, np.nan)
        terminal_monitor_history = []
        overshoot_count = 0
        priority_changes = priority_change_cycles = zero_events = 0
        minimum_candidates = np.iinfo(np.int64).max
        minimum_pair = minimum_static = minimum_dynamic = np.inf
        pair_violations = static_violations = dynamic_violations = 0
        previous_priority = None
        failure = None
        initial_time = float(mission_time)

        for cycle in range(int(cycles)):
            cycle_start = time.perf_counter()
            monitor = tuple(self.terminal_feasibility(
                positions[i], velocities[i], goals[i]) for i in range(count))
            terminal_monitor_history.append(monitor)
            if self.arrival_handling == 'hold':
                active_ids = np.flatnonzero(arrival_cycles < 0)
                plans = [None] * count
                if len(active_ids):
                    active_plans, _, base_timing = self.batch_planner.plan(
                        [self.planners[i] for i in active_ids],
                        [states[i] for i in active_ids], mission_time)
                    for uav, plan in zip(active_ids, active_plans):
                        plans[int(uav)] = plan
                else:
                    base_timing = self._zero_timing()
                for uav in np.flatnonzero(arrival_cycles >= 0):
                    plans[int(uav)] = self._arrived_plan(
                        self.planners[int(uav)], states[int(uav)], mission_time)
                plans = tuple(plans)
            else:
                plans, _, base_timing = self.batch_planner.plan(
                    self.planners, states, mission_time)
            coordinated = self.coordinator.coordinate(plans, states, mission_time)
            elapsed = time.perf_counter() - cycle_start
            if not coordinated.success:
                failure = self._snapshot(cycle, mission_time, coordinated.reason,
                                         states, base_timing, coordinated)
                if coordinated.reason == 'no coordinated candidate':
                    failure['reservation_diagnostic'] = diagnose_reservation_failure(
                        plans, states, coordinated,
                        self.coordinator.collision_checker,
                        arrival_cycles >= 0)
                break
            if elapsed >= self.cycle_deadline:
                failure = self._snapshot(cycle, mission_time,
                    'complete swarm cycle deadline exceeded', states,
                    base_timing, coordinated)
                failure['complete_cycle_time'] = elapsed
                break

            segment_p, segment_v, segment_a = [], [], []
            for uav, index in enumerate(coordinated.selected_indices):
                p, v, a = evaluate_quintics(
                    plans[uav].primitive_batch.coefficients[index:index + 1],
                    self.execution_times)
                segment_p.append(p[0]); segment_v.append(v[0]); segment_a.append(a[0])
                key = int(index)
                selected_counts[uav][key] = selected_counts[uav].get(key, 0) + 1
            segment_p = np.asarray(segment_p)
            segment_v = np.asarray(segment_v)
            segment_a = np.asarray(segment_a)
            absolute_times = mission_time + self.execution_times
            audit = self.evaluator.audit_segment(segment_p, absolute_times)
            minimum_pair = min(minimum_pair, audit.minimum_uav_distance)
            minimum_static = min(minimum_static, audit.minimum_static_clearance)
            minimum_dynamic = min(minimum_dynamic, audit.minimum_dynamic_clearance)
            pair_violations += audit.uav_violation_count
            static_violations += audit.static_violation_count
            dynamic_violations += audit.dynamic_violation_count
            if (audit.uav_violation_count or audit.static_violation_count or
                    audit.dynamic_violation_count):
                failure = self._snapshot(cycle, mission_time,
                    'executed segment safety audit failed', states,
                    base_timing, coordinated, audit)
                break

            priority = coordinated.priority_order
            if previous_priority is not None:
                changed = int(np.count_nonzero(priority != previous_priority))
                priority_changes += changed
                priority_change_cycles += int(changed > 0)
            previous_priority = priority.copy()
            counts = coordinated.base_valid_counts
            zero_events += int(np.count_nonzero(counts == 0))
            minimum_candidates = min(minimum_candidates, int(np.min(counts)))
            records.append({
                'cycle': cycle, 'mission_time': mission_time,
                'complete_cycle_time': elapsed,
                'base_planning_time': base_timing.total,
                'coordination_time': coordinated.planning_time,
                'priority_order': priority.copy(),
                'selected_indices': coordinated.selected_indices.copy(),
                'base_valid_counts': counts.copy(),
                'minimum_uav_distance': audit.minimum_uav_distance,
                'minimum_static_clearance': audit.minimum_static_clearance,
                'minimum_dynamic_clearance': audit.minimum_dynamic_clearance,
                'arrival_ids': np.flatnonzero(arrival_cycles >= 0).copy(),
                'terminal_feasibility': monitor,
            })
            positions = segment_p[:, -1, :].copy()
            velocities = segment_v[:, -1, :].copy()
            accelerations = segment_a[:, -1, :].copy()
            mission_time += self.execution_horizon
            states = [(positions[i], velocities[i], accelerations[i])
                      for i in range(count)]
            newly_arrived = ((arrival_cycles < 0) &
                (np.linalg.norm(positions - goals, axis=1) <=
                 config.GOAL_TOLERANCE_ASSUMED))
            arrival_cycles[newly_arrived] = cycle
            arrival_positions[newly_arrived] = positions[newly_arrived]
            if self.arrival_handling == 'hold' and np.any(newly_arrived):
                velocities[newly_arrived] = 0.
                accelerations[newly_arrived] = 0.
                states = [(positions[i], velocities[i], accelerations[i])
                          for i in range(count)]
            overshoot_count += int(np.count_nonzero(
                (arrival_cycles >= 0) &
                (np.linalg.norm(positions - goals, axis=1) >
                 config.GOAL_TOLERANCE_ASSUMED)))

        completed = len(records)
        success = failure is None and completed == int(cycles)
        if minimum_candidates == np.iinfo(np.int64).max:
            minimum_candidates = 0
        entry_snapshots = [planner.terminal_policy.entry_snapshot
                           for planner in self.planners]
        terminal_statistics = {
            'monitor_history': tuple(terminal_monitor_history),
            'entry_distances': np.asarray([
                np.nan if item is None else item['goal_distance']
                for item in entry_snapshots]),
            'entry_speeds': np.asarray([
                np.nan if item is None else item['speed'] for item in entry_snapshots]),
            'arrival_positions': arrival_positions.copy(),
            'capture_success_ratio': float(np.count_nonzero(arrival_cycles >= 0)) /
                                     float(count)}
        return SwarmRollingResult(
            success, 'success' if success else failure['reason'], completed,
            float(mission_time - initial_time), positions, velocities, accelerations,
            tuple(records), failure, minimum_pair, minimum_static, minimum_dynamic,
            pair_violations, static_violations, dynamic_violations,
            priority_changes, priority_change_cycles,
            tuple(dict(x) for x in selected_counts), zero_events,
            minimum_candidates, arrival_cycles, terminal_statistics,
            self.arrival_handling, overshoot_count)
