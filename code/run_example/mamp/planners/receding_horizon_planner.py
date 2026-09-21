"""Single-UAV orchestration of the frozen Phase 2--5 planning modules."""

from dataclasses import dataclass
import time

import numpy as np

from ..configs import subject3_config as config
from .dynamic_primitive_selector import DynamicPrimitiveSelector
from .primitive_selector import StaticPrimitiveSelector
from .quintic_primitive import QuinticPrimitiveGenerator
from .terminal_policy import TerminalPolicy


@dataclass
class RollingPlanResult:
    success: bool
    reason: str
    selected_index: object
    selected_coefficients: object
    primitive_batch: object
    static_selection: object
    dynamic_selection: object
    guide_direction: np.ndarray
    guide_target: np.ndarray
    guide_segment_index: int
    guide_query_time: float
    primitive_generation_time: float
    phase4_time: float
    phase5_time: float
    orchestrator_overhead: float
    planning_time: float
    failure_snapshot: object
    mode: str = 'CRUISE'
    remaining_guide_distance: float = float('inf')
    goal_distance: float = float('inf')
    terminal_reference_speed: object = None
    terminal_context: object = None
    terminal_ranking: object = None
    turn_diagnostics: object = None


class RecedingHorizonPlanner(object):
    """Plan one receding-horizon cycle without executing or retaining batches."""

    def __init__(self, guide_tracker, obstacles, generator=None,
                 static_selector=None, dynamic_selector=None,
                 global_astar_calls=0, goal=None, terminal_capture_speed=None,
                 legacy_terminal_envelope=False, terminal_release_delay=0.0,
                 terminal_wait_reference_speed=None,
                 terminal_goal_direction_distance=None):
        self.guide_tracker = guide_tracker
        self.generator = generator or QuinticPrimitiveGenerator()
        self.static_selector = static_selector or StaticPrimitiveSelector(obstacles)
        self.dynamic_selector = dynamic_selector or DynamicPrimitiveSelector(obstacles)
        self.obstacles = tuple(obstacles)
        self.plan_step_calls = 0
        self.global_astar_calls = int(global_astar_calls)
        self.goal = None if goal is None else np.asarray(goal, dtype='float64')
        self.terminal_policy = (None if goal is None else
                                TerminalPolicy(self.goal, self.generator,
                                               terminal_capture_speed,
                                               legacy_terminal_envelope,
                                               terminal_release_delay,
                                               terminal_wait_reference_speed,
                                               terminal_goal_direction_distance))

    def replace_goal(self, goal, guide_tracker=None, reset_terminal=False):
        """Update the planning goal; reset the goal-version latch only on a switch."""
        self.goal = np.asarray(goal, dtype='float64').copy()
        if guide_tracker is not None:
            self.guide_tracker = guide_tracker
        if self.terminal_policy is None or reset_terminal:
            capture = (config.TERMINAL_CAPTURE_SPEED if self.terminal_policy is None
                       else self.terminal_policy.capture_speed)
            legacy = (False if self.terminal_policy is None else
                      self.terminal_policy.legacy_envelope)
            release_delay = (0.0 if self.terminal_policy is None else
                             self.terminal_policy.release_delay)
            wait_speed = (None if self.terminal_policy is None else
                          self.terminal_policy.wait_reference_speed)
            goal_direction_distance = (
                None if self.terminal_policy is None else
                self.terminal_policy.goal_direction_distance)
            self.terminal_policy = TerminalPolicy(self.goal, self.generator,
                                                  capture, legacy,
                                                  release_delay, wait_speed,
                                                  goal_direction_distance)
        else:
            self.terminal_policy.goal = self.goal.copy()

    def _nearest_obstacle(self, position, dynamic, mission_time=0.0):
        candidates = [item for item in self.obstacles
                      if (not item.is_static) == dynamic]
        if not candidates:
            return None
        values = []
        for item in candidates:
            if dynamic:
                center, yaw = item.predict_pose(float(mission_time))
                original_position = item.pos_global_frame
                original_angle = getattr(item, 'angle', 0.0)
                item.pos_global_frame = np.asarray(center)
                if item.shape in ('cube', 'rotated_cube'):
                    item.angle = float(yaw)
                distance = float(np.sqrt(item.distance_sq_to_point(position)))
                item.pos_global_frame = original_position
                if item.shape in ('cube', 'rotated_cube'):
                    item.angle = original_angle
            else:
                distance = float(np.sqrt(item.distance_sq_to_point(position)))
            values.append((distance, int(item.competition_id)))
        distance, obstacle_id = min(values)
        return {'obstacle_id': obstacle_id, 'clearance': distance}

    def _snapshot(self, mission_time, position, velocity, acceleration,
                  direction, target, batch, static_selection,
                  dynamic_selection, selected_index=None):
        count = 0 if batch is None else len(batch.positions)
        feasible = 0 if batch is None else int(np.count_nonzero(batch.feasible_mask))
        rejected = {} if batch is None else {
            'speed': int(np.count_nonzero(~batch.speed_ok)),
            'altitude': int(np.count_nonzero(~batch.altitude_ok)),
            'xz_acc': int(np.count_nonzero(~batch.xz_acc_ok)),
            'y_up': int(np.count_nonzero(~batch.y_up_ok)),
            'y_down': int(np.count_nonzero(~batch.y_down_ok)),
        }
        snapshot = {
            'mission_time': float(mission_time), 'position': position.copy(),
            'velocity': velocity.copy(), 'acceleration': acceleration.copy(),
            'guide_direction': direction.copy(), 'guide_target': target.copy(),
            'guide_segment_index': int(self.guide_tracker.current_segment_index),
            'candidate_count': count, 'phase3_feasible_count': feasible,
            'phase3_rejected': rejected,
            'static_safe_count': (0 if static_selection is None else
                                  int(np.count_nonzero(static_selection.static_safe_mask))),
            'phase4_valid_count': (0 if static_selection is None else
                                   len(static_selection.ranked_indices)),
            'intersection_count': (0 if static_selection is None else
                int(np.count_nonzero(static_selection.fine_dynamic_mask &
                                     static_selection.static_safe_mask))),
            'dynamic_safe_count': (0 if dynamic_selection is None else
                int(np.count_nonzero(dynamic_selection.final_valid_mask))),
            'nearest_static_obstacle': self._nearest_obstacle(position, False,
                                                              mission_time),
            'nearest_dynamic_obstacle': self._nearest_obstacle(position, True,
                                                               mission_time),
            'selected_candidate': None,
        }
        if selected_index is not None:
            index = int(selected_index)
            snapshot['selected_candidate'] = {
                'index': index,
                'terminal_speed': float(batch.terminal_speeds[index]),
                'lateral_offset': float(batch.lateral_offsets[index]),
                'vertical_offset': float(batch.vertical_offsets[index]),
            }
        return snapshot

    def plan_step(self, position, velocity, acceleration, mission_time):
        total_start = time.perf_counter()
        self.plan_step_calls += 1
        position = np.asarray(position, dtype='float64')
        velocity = np.asarray(velocity, dtype='float64')
        acceleration = np.asarray(acceleration, dtype='float64')

        guide_start = time.perf_counter()
        remaining = self.guide_tracker.get_remaining_path_length(position)
        direction = self.guide_tracker.get_turn_aware_direction(position, velocity)
        turn_diagnostics = self.guide_tracker.last_turn_diagnostics
        target = self.guide_tracker.last_target.copy()
        goal_distance = (float('inf') if self.goal is None else
                         float(np.linalg.norm(self.goal - position)))
        terminal = (False if self.terminal_policy is None else
                    self.terminal_policy.update_mode(
                        remaining, mission_time, position,
                        np.linalg.norm(velocity)))
        mode = 'TERMINAL' if terminal else 'CRUISE'
        if terminal:
            on_last_segment = (self.guide_tracker.current_segment_index >=
                               len(self.guide_tracker.waypoints) - 2)
            direction = self.terminal_policy.terminal_direction(
                position, direction, remaining, on_last_segment)
        guide_time = time.perf_counter() - guide_start

        terminal_context = None
        if terminal:
            terminal_context = self.terminal_policy.generate(
                position, velocity, acceleration, direction, remaining)
            batch = terminal_context.batch
            generation_time = terminal_context.generation_time
        else:
            generation_start = time.perf_counter()
            batch = self.generator.generate(position, velocity, acceleration, direction)
            generation_time = time.perf_counter() - generation_start
        if not np.any(batch.feasible_mask):
            reason = 'no dynamically feasible primitive'
            snapshot = self._snapshot(mission_time, position, velocity,
                                      acceleration, direction, target, batch,
                                      None, None)
            snapshot.update({'mode': mode, 'remaining_guide_distance': remaining,
                             'goal_distance': goal_distance,
                             'turn_diagnostics': turn_diagnostics})
            elapsed = time.perf_counter() - total_start
            return RollingPlanResult(False, reason, None, None, batch, None, None,
                                     direction, target,
                                     self.guide_tracker.current_segment_index,
                                     guide_time, generation_time, 0.0, 0.0,
                                     elapsed - guide_time - generation_time,
                                     elapsed, snapshot, mode, remaining,
                                     goal_distance,
                                     None if terminal_context is None else
                                     terminal_context.reference_speed,
                                     terminal_context, None, turn_diagnostics)

        static_start = time.perf_counter()
        static_selection = self.static_selector.select(batch, position, direction)
        phase4_time = time.perf_counter() - static_start
        if not static_selection.success:
            snapshot = self._snapshot(mission_time, position, velocity,
                                      acceleration, direction, target, batch,
                                      static_selection, None)
            snapshot.update({'mode': mode, 'remaining_guide_distance': remaining,
                             'goal_distance': goal_distance,
                             'turn_diagnostics': turn_diagnostics})
            elapsed = time.perf_counter() - total_start
            return RollingPlanResult(
                False, static_selection.reason, None, None, batch,
                static_selection, None, direction, target,
                self.guide_tracker.current_segment_index, guide_time,
                generation_time, phase4_time, 0.0,
                elapsed - guide_time - generation_time - phase4_time,
                elapsed, snapshot, mode, remaining, goal_distance,
                None if terminal_context is None else
                terminal_context.reference_speed, terminal_context, None,
                turn_diagnostics)

        phase5_start = time.perf_counter()
        dynamic_selection = self.dynamic_selector.select(static_selection,
                                                         mission_time)
        phase5_time = time.perf_counter() - phase5_start
        if not dynamic_selection.success:
            snapshot = self._snapshot(mission_time, position, velocity,
                                      acceleration, direction, target, batch,
                                      static_selection, dynamic_selection)
            snapshot.update({'mode': mode, 'remaining_guide_distance': remaining,
                             'goal_distance': goal_distance,
                             'turn_diagnostics': turn_diagnostics})
            elapsed = time.perf_counter() - total_start
            return RollingPlanResult(
                False, dynamic_selection.reason, None, None, batch,
                static_selection, dynamic_selection, direction, target,
                self.guide_tracker.current_segment_index, guide_time,
                generation_time, phase4_time, phase5_time,
                elapsed - guide_time - generation_time - phase4_time - phase5_time,
                elapsed, snapshot, mode, remaining, goal_distance,
                None if terminal_context is None else
                terminal_context.reference_speed, terminal_context, None,
                turn_diagnostics)

        terminal_ranking = None
        if terminal:
            terminal_ranking = self.terminal_policy.rank(
                dynamic_selection, terminal_context, goal_distance)
            selected = terminal_ranking.best_index
        else:
            selected = int(dynamic_selection.ranked_indices[0])
        elapsed = time.perf_counter() - total_start
        return RollingPlanResult(
            True, 'success', selected, batch.coefficients[selected], batch,
            static_selection, dynamic_selection, direction, target,
            self.guide_tracker.current_segment_index, guide_time,
            generation_time, phase4_time, phase5_time,
            elapsed - guide_time - generation_time - phase4_time - phase5_time,
            elapsed, None, mode, remaining, goal_distance,
            None if terminal_context is None else terminal_context.reference_speed,
            terminal_context, terminal_ranking, turn_diagnostics)
