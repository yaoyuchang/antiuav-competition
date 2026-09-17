"""Execute short analytic quintic prefixes in a single-UAV rolling loop."""

from dataclasses import dataclass
import math

import numpy as np

from ..collision.dynamic_primitive_collision import distance_to_predicted_obstacle
from ..configs import subject3_config as config
from ..planners.quintic_primitive import evaluate_quintics
from ..planners.global_guide import GlobalGuideTracker


@dataclass
class RollingMissionResult:
    success: bool
    reason: str
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    cycle_records: tuple
    failure_snapshot: object
    replanning_calls: int
    global_astar_calls: int
    mission_time: float
    distance_traveled: float
    average_speed: float
    max_speed: float
    min_static_clearance: float
    min_dynamic_clearance: float
    max_xz_acceleration: float
    max_ay: float
    min_ay: float
    curvature_integral: float
    peak_jerk: float
    mean_jerk: float
    max_position_splice_error: float
    max_velocity_splice_error: float
    max_acceleration_splice_error: float
    mean_planning_time: float
    median_planning_time: float
    p95_planning_time: float
    p99_planning_time: float
    max_planning_time: float
    speed_target_changes: int
    lateral_sign_changes: int
    vertical_offset_changes: int
    tracker_indices: np.ndarray
    official_static_safe: bool
    official_dynamic_safe: bool
    planner_guard_satisfied: bool
    arrived: bool = False
    arrival_time: object = None
    arrival_position: object = None
    arrival_distance_error: object = None
    arrival_speed: object = None
    cruise_cycle_count: int = 0
    terminal_cycle_count: int = 0
    terminal_entry: object = None
    minimum_goal_distance_before_arrival: object = None
    overshot_goal_before_arrival: bool = False
    minimum_candidate_intersection: int = 0
    p5_candidate_intersection: float = 0.0
    median_candidate_intersection: float = 0.0
    goal_history_times: object = None
    observed_goal_history: object = None
    filtered_goal_history: object = None


class SingleUAVRollingSimulator(object):
    def __init__(self, planner, obstacles, execution_horizon=None,
                 record_dt=None, goal_manager=None, global_guide_planner=None):
        self.planner = planner
        self.obstacles = tuple(obstacles)
        self.static_obstacles = tuple(item for item in obstacles if item.is_static)
        self.dynamic_obstacles = tuple(item for item in obstacles if not item.is_static)
        self.goal_manager = goal_manager
        self.global_guide_planner = global_guide_planner
        self.execution_horizon = float(
            config.EXECUTION_HORIZON if execution_horizon is None
            else execution_horizon)
        self.record_dt = float(config.EXECUTION_RECORD_DT if record_dt is None
                               else record_dt)
        if not 0.0 < self.execution_horizon < config.PRIMITIVE_HORIZON:
            raise ValueError('execution_horizon must be in (0, primitive horizon)')
        intervals = int(math.ceil(self.execution_horizon / self.record_dt))
        self.execution_times = np.linspace(0.0, self.execution_horizon,
                                           intervals + 1)

    def _audit_segment(self, positions, absolute_times, dynamic_guard):
        static_min = np.inf
        for obstacle in self.static_obstacles:
            static_min = min(static_min, float(np.min(np.sqrt(
                obstacle.distance_sq_to_points(positions)))))
        dynamic_min = np.inf
        for obstacle in self.dynamic_obstacles:
            centers, yaw = obstacle.predict_pose(absolute_times)
            distance = distance_to_predicted_obstacle(
                positions[None, :, :], obstacle, centers, yaw)[0]
            dynamic_min = min(dynamic_min, float(np.min(distance)))
        official_static = static_min >= config.OBS_SAFE_DISTANCE - 1.0e-8
        official_dynamic = dynamic_min >= config.OBS_SAFE_DISTANCE - 1.0e-8
        guard_ok = (static_min >= config.OBS_SAFE_DISTANCE +
                    self.planner.static_selector.collision_checker.collision_guard - 1.0e-8 and
                    dynamic_min >= config.OBS_SAFE_DISTANCE + dynamic_guard - 1.0e-8)
        return static_min, dynamic_min, official_static, official_dynamic, guard_ok

    @staticmethod
    def _evaluate_one(coefficients, tau):
        position, velocity, acceleration = evaluate_quintics(
            coefficients[None, :, :], np.array([tau], dtype='float64'))
        return position[0, 0], velocity[0, 0], acceleration[0, 0]

    def _first_arrival(self, coefficients, positions, goal, mission_time=0.0):
        callable_goal = callable(goal)
        goals = (np.asarray([goal(mission_time + tau)
                             for tau in self.execution_times])
                 if callable_goal else goal)
        distances = np.linalg.norm(positions - goals, axis=1)
        inside = np.flatnonzero(distances <= config.GOAL_TOLERANCE_ASSUMED)
        if not len(inside):
            return None
        index = int(inside[0])
        if index == 0:
            p, v, a = self._evaluate_one(coefficients, self.execution_times[0])
            return self.execution_times[0], p, v, a, index
        low, high = self.execution_times[index - 1], self.execution_times[index]
        for _ in range(config.ARRIVAL_BISECTION_ITERATIONS):
            middle = 0.5 * (low + high)
            p, _, _ = self._evaluate_one(coefficients, middle)
            current_goal = goal(mission_time + middle) if callable_goal else goal
            if np.linalg.norm(p - current_goal) <= config.GOAL_TOLERANCE_ASSUMED:
                high = middle
            else:
                low = middle
        p, v, a = self._evaluate_one(coefficients, high)
        return high, p, v, a, index

    def run(self, position, velocity=None, acceleration=None, mission_time=0.0,
            end_x=None, max_time=None, max_cycles=None, goal=None):
        position = np.asarray(position, dtype='float64').copy()
        velocity = np.zeros(3) if velocity is None else np.asarray(velocity, dtype='float64').copy()
        acceleration = (np.zeros(3) if acceleration is None else
                        np.asarray(acceleration, dtype='float64').copy())
        mission_time = float(mission_time)
        initial_time = mission_time
        end_x = config.ROLLING_TEST_END_X if end_x is None else float(end_x)
        max_time = config.ROLLING_TEST_MAX_TIME if max_time is None else float(max_time)
        max_cycles = 1000000 if max_cycles is None else int(max_cycles)
        goal = None if goal is None else np.asarray(goal, dtype='float64')
        history_t, history_p, history_v, history_a = [], [], [], []
        goal_history_t, observed_goal_history, filtered_goal_history = [], [], []
        records = []
        tracker_indices = []
        static_minimum, dynamic_minimum = np.inf, np.inf
        official_static_safe = True
        official_dynamic_safe = True
        guard_satisfied = True
        splice_p, splice_v, splice_a = [], [], []
        failure = None
        reason = 'test horizon reached'
        success = True
        arrived = bool(goal is not None and
                       np.linalg.norm(position - goal) <= config.GOAL_TOLERANCE_ASSUMED)
        arrival_time = mission_time if arrived else None
        arrival_position = position.copy() if arrived else None
        arrival_speed = float(np.linalg.norm(velocity)) if arrived else None
        minimum_goal_distance = (None if goal is None else
                                 float(np.linalg.norm(position - goal)))
        overshot = False

        while ((goal is not None or position[0] < end_x) and not arrived and
               mission_time - initial_time < max_time and
               len(records) < max_cycles):
            goal_update = None
            switch_guide = None
            if self.goal_manager is not None:
                goal_update = self.goal_manager.update(position, mission_time)
                if goal_update.switched:
                    if self.global_guide_planner is None:
                        success, reason = False, 'goal switch requires global guide planner'
                        failure = {'mission_time': mission_time,
                                   'position': position.copy(), 'reason': reason}
                        break
                    switch_guide = self.global_guide_planner.plan(
                        position, goal_update.base_goal)
                    if not switch_guide.success:
                        success, reason = False, 'switched global guide failed'
                        failure = {'mission_time': mission_time,
                                   'position': position.copy(),
                                   'reason': switch_guide.reason}
                        break
                    new_tracker = GlobalGuideTracker(
                        switch_guide.waypoints, turn_aware_enabled=True,
                        turn_accel_ref_ratio=config.GUIDE_TURN_ACCEL_REF_RATIO)
                    self.planner.global_astar_calls += 1
                    self.planner.replace_goal(goal_update.filtered_goal,
                                              new_tracker, reset_terminal=True)
                else:
                    self.planner.replace_goal(goal_update.filtered_goal)
                arrival_goal = self.goal_manager.arrival_goal.copy()
            else:
                arrival_goal = goal
            plan = self.planner.plan_step(position, velocity, acceleration,
                                          mission_time)
            if not plan.success:
                success, reason, failure = False, plan.reason, plan.failure_snapshot
                break
            index = plan.selected_index
            coefficients = plan.selected_coefficients[None, :, :]
            segment_p, segment_v, segment_a = evaluate_quintics(
                coefficients, self.execution_times)
            segment_p, segment_v, segment_a = segment_p[0], segment_v[0], segment_a[0]
            splice_p.append(float(np.linalg.norm(segment_p[0] - position)))
            splice_v.append(float(np.linalg.norm(segment_v[0] - velocity)))
            splice_a.append(float(np.linalg.norm(segment_a[0] - acceleration)))
            arrival_source = (self.goal_manager.true_goal_at
                              if self.goal_manager is not None and
                              self.goal_manager.arrival_goal_mode == 'observed'
                              else arrival_goal)
            arrival = None if arrival_source is None else self._first_arrival(
                plan.selected_coefficients, segment_p, arrival_source, mission_time)
            if arrival is not None:
                tau_arrival, arrived_p, arrived_v, arrived_a, arrival_index = arrival
                segment_p = np.vstack((segment_p[:arrival_index], arrived_p))
                segment_v = np.vstack((segment_v[:arrival_index], arrived_v))
                segment_a = np.vstack((segment_a[:arrival_index], arrived_a))
                executed_relative_times = np.r_[
                    self.execution_times[:arrival_index], tau_arrival]
            else:
                executed_relative_times = self.execution_times
            absolute_times = mission_time + executed_relative_times
            static_value, dynamic_value, static_ok, dynamic_ok, guard_ok = (
                self._audit_segment(segment_p, absolute_times,
                                    plan.dynamic_selection.dynamic_collision_guard))
            static_minimum = min(static_minimum, static_value)
            dynamic_minimum = min(dynamic_minimum, dynamic_value)
            official_static_safe &= static_ok
            official_dynamic_safe &= dynamic_ok
            guard_satisfied &= guard_ok

            start_index = 0 if not history_t else 1
            history_t.append(absolute_times[start_index:])
            history_p.append(segment_p[start_index:])
            history_v.append(segment_v[start_index:])
            history_a.append(segment_a[start_index:])
            if self.goal_manager is not None:
                sample_times = absolute_times[start_index:]
                goal_history_t.append(sample_times)
                observed_goal_history.append(np.asarray([
                    self.goal_manager.true_goal_at(value) for value in sample_times]))
                filtered_goal_history.append(np.tile(
                    goal_update.filtered_goal, (len(sample_times), 1)))
            phase4, phase5, batch = (plan.static_selection,
                                     plan.dynamic_selection,
                                     plan.primitive_batch)
            records.append({
                'mission_time': mission_time, 'position': position.copy(),
                'velocity': velocity.copy(),
                'mode': plan.mode, 'goal_distance': plan.goal_distance,
                'remaining_guide_distance': plan.remaining_guide_distance,
                'terminal_reference_speed': plan.terminal_reference_speed,
                'speed': float(np.linalg.norm(velocity)), 'selected_index': index,
                'terminal_speed': float(batch.terminal_speeds[index]),
                'lateral_offset': float(batch.lateral_offsets[index]),
                'vertical_offset': float(batch.vertical_offsets[index]),
                'progress': float(phase4.progress[index]),
                'static_clearance': float(phase4.min_static_clearance[index]),
                'dynamic_clearance': float(phase5.min_dynamic_clearance[index]),
                'closest_dynamic_obstacle_id': int(
                    phase5.closest_dynamic_obstacle_id[index]),
                'time_of_min_dynamic_clearance': float(
                    phase5.time_of_min_dynamic_clearance[index]),
                'phase4_count': len(phase4.ranked_indices),
                'candidate_count': len(batch.positions),
                'phase3_feasible_count': int(np.count_nonzero(batch.feasible_mask)),
                'intersection_count': int(np.count_nonzero(
                    phase4.fine_dynamic_mask & phase4.static_safe_mask)),
                'phase5_count': len(phase5.ranked_indices),
                'endpoint_goal_distance': (float('nan') if goal is None else
                    float(np.linalg.norm(batch.terminal_positions[index] - goal))),
                'exact_goal_candidate': bool(
                    plan.terminal_context is not None and
                    plan.terminal_context.exact_goal_mask[index]),
                'candidate_type': ('cruise' if plan.terminal_context is None else
                                   str(plan.terminal_context.candidate_types[index])),
                'capture_boundary_feasible_count': (0 if plan.terminal_context is None else
                    int(np.count_nonzero(
                        plan.terminal_context.capture_boundary_mask &
                        batch.feasible_mask))),
                'total_cost': float(
                    phase5.total_cost[index] if plan.terminal_ranking is None
                    else plan.terminal_ranking.total_cost[index]),
                'guide_direction': plan.guide_direction.copy(),
                'guide_target': plan.guide_target.copy(),
                'guide_segment_index': plan.guide_segment_index,
                'turn_diagnostics': plan.turn_diagnostics,
                'guide_query_time': plan.guide_query_time,
                'phase3_time': plan.primitive_generation_time,
                'phase4_time': plan.phase4_time, 'phase5_time': plan.phase5_time,
                'orchestrator_overhead': plan.orchestrator_overhead,
                'planning_time': plan.planning_time,
                'goal_version': (0 if goal_update is None else
                                 goal_update.goal_version),
                'base_goal': (None if goal_update is None else
                              goal_update.base_goal.copy()),
                'observed_goal': (None if goal_update is None else
                                  goal_update.observed_goal.copy()),
                'filtered_goal': (None if goal_update is None else
                                  goal_update.filtered_goal.copy()),
                'goal_update_time': (0.0 if goal_update is None else
                                     goal_update.update_time),
                'goal_switched_this_cycle': bool(
                    goal_update is not None and goal_update.switched),
                'switch_global_guide': switch_guide,
            })
            tracker_indices.append(plan.guide_segment_index)
            position, velocity, acceleration = (segment_p[-1].copy(),
                                                segment_v[-1].copy(),
                                                segment_a[-1].copy())
            mission_time = float(absolute_times[-1])
            if arrival_goal is not None:
                distance_to_goal = float(np.linalg.norm(position - arrival_goal))
                minimum_goal_distance = min(minimum_goal_distance,
                                            distance_to_goal)
                overshot |= bool(np.dot(position - goal, plan.guide_direction) > 0.0)
            if arrival is not None:
                arrived = True
                arrival_time = mission_time
                arrival_position = position.copy()
                arrival_speed = float(np.linalg.norm(velocity))
                reason = 'arrived within goal tolerance'
                break

        if success:
            if arrived:
                reason = 'arrived within goal tolerance'
            elif goal is not None:
                success = False
                reason = 'goal not reached before rolling limit'
            elif goal is None and position[0] >= end_x:
                reason = 'test end x reached'
            elif len(records) >= max_cycles:
                reason = 'maximum rolling cycles reached'
            else:
                reason = 'maximum rolling test time reached'

        if history_t:
            times = np.concatenate(history_t)
            positions = np.vstack(history_p)
            velocities = np.vstack(history_v)
            accelerations = np.vstack(history_a)
        else:
            times = np.array([initial_time])
            positions, velocities, accelerations = (position[None, :],
                                                     velocity[None, :],
                                                     acceleration[None, :])
        speeds = np.linalg.norm(velocities, axis=1)
        xz_acc = np.linalg.norm(accelerations[:, [0, 2]], axis=1)
        distance = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))
        duration = max(float(times[-1] - times[0]), 0.0)
        cross = np.linalg.norm(np.cross(velocities, accelerations), axis=1)
        curvature = np.divide(cross, speeds ** 3, out=np.zeros_like(cross),
                              where=speeds > 1.0e-8)
        curvature_integral = float(np.trapz(curvature ** 2 * speeds, times))
        if len(times) > 1:
            jerk = np.linalg.norm(np.diff(accelerations, axis=0) /
                                  np.diff(times)[:, None], axis=1)
        else:
            jerk = np.zeros(1)
        planning_times = np.asarray([item['planning_time'] for item in records])
        terminal_speeds = np.asarray([item['terminal_speed'] for item in records])
        lateral_signs = np.sign([item['lateral_offset'] for item in records])
        vertical_offsets = np.asarray([item['vertical_offset'] for item in records])
        intersections = np.asarray([item['intersection_count'] for item in records],
                                   dtype='int64')
        if failure is not None and 'intersection_count' in failure:
            intersections = np.r_[intersections, int(failure['intersection_count'])]
        cruise_count = sum(item['mode'] == 'CRUISE' for item in records)
        terminal_count = sum(item['mode'] == 'TERMINAL' for item in records)
        statistic = lambda function, default=0.0: (
            float(function(planning_times)) if len(planning_times) else default)
        return RollingMissionResult(
            success, reason, times, positions, velocities, accelerations,
            tuple(records), failure, len(records),
            self.planner.global_astar_calls, mission_time, distance,
            distance / duration if duration > 0.0 else 0.0,
            float(np.max(speeds)), static_minimum, dynamic_minimum,
            float(np.max(xz_acc)), float(np.max(accelerations[:, 1])),
            float(np.min(accelerations[:, 1])), curvature_integral,
            float(np.max(jerk)), float(np.mean(jerk)),
            max(splice_p or [0.0]), max(splice_v or [0.0]),
            max(splice_a or [0.0]), statistic(np.mean), statistic(np.median),
            statistic(lambda values: np.percentile(values, 95.0)),
            statistic(lambda values: np.percentile(values, 99.0)),
            statistic(np.max),
            int(np.count_nonzero(np.diff(terminal_speeds))) if len(records) > 1 else 0,
            int(np.count_nonzero(np.diff(lateral_signs))) if len(records) > 1 else 0,
            int(np.count_nonzero(np.diff(vertical_offsets))) if len(records) > 1 else 0,
            np.asarray(tracker_indices, dtype='int64'), official_static_safe,
            official_dynamic_safe, guard_satisfied, arrived, arrival_time,
            arrival_position,
            None if arrival_position is None else float(np.linalg.norm(
                arrival_position - (self.goal_manager.true_goal_at(arrival_time)
                                     if self.goal_manager is not None and
                                     self.goal_manager.arrival_goal_mode == 'observed'
                                     else (self.goal_manager.arrival_goal
                                           if self.goal_manager is not None else goal)))),
            arrival_speed, cruise_count,
            terminal_count,
            None if self.planner.terminal_policy is None else
            self.planner.terminal_policy.entry_snapshot,
            minimum_goal_distance, overshot,
            int(np.min(intersections)) if len(intersections) else 0,
            float(np.percentile(intersections, 5.0)) if len(intersections) else 0.0,
            float(np.median(intersections)) if len(intersections) else 0.0,
            (np.concatenate(goal_history_t) if goal_history_t else np.empty(0)),
            (np.vstack(observed_goal_history) if observed_goal_history else
             np.empty((0, 3))),
            (np.vstack(filtered_goal_history) if filtered_goal_history else
             np.empty((0, 3))))
