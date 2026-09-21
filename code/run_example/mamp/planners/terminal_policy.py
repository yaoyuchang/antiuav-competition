"""Latched terminal primitive generation and goal-oriented re-ranking."""

from dataclasses import dataclass
import math
import time

import numpy as np

from ..configs import subject3_config as config
from .quintic_primitive import PrimitiveBatch
from .quintic_primitive import closed_form_quintic_coefficients
from .quintic_primitive import evaluate_quintics


@dataclass
class TerminalPrimitiveContext:
    batch: PrimitiveBatch
    exact_goal_mask: np.ndarray
    capture_boundary_mask: np.ndarray
    candidate_types: np.ndarray
    reference_speed: float
    offset_scale: float
    direction: np.ndarray
    generation_time: float


@dataclass
class TerminalRanking:
    goal_cost: np.ndarray
    velocity_reference_cost: np.ndarray
    total_cost: np.ndarray
    ranked_indices: np.ndarray
    best_index: object
    alpha: float
    ranking_time: float


class TerminalPolicy(object):
    def __init__(self, goal, generator, capture_speed=None, legacy_envelope=False,
                 release_delay=0.0, wait_reference_speed=None,
                 goal_direction_distance=None):
        self.goal = np.asarray(goal, dtype='float64')
        if self.goal.shape != (3,):
            raise ValueError('goal must have shape (3,)')
        self.generator = generator
        self.capture_speed = float(config.TERMINAL_CAPTURE_SPEED if capture_speed is None
                                   else capture_speed)
        if not 0.0 <= self.capture_speed <= config.V_MAX:
            raise ValueError('capture_speed must be in [0, V_MAX]')
        self.legacy_envelope = bool(legacy_envelope)
        self.terminal_mode = False
        self.entry_snapshot = None
        self.release_delay = float(release_delay)
        self.wait_reference_speed = (None if wait_reference_speed is None else
                                     float(wait_reference_speed))
        self.release_time = None
        self.current_mission_time = 0.0
        self.goal_direction_distance = float(
            config.TERMINAL_GOAL_DIRECTION_DISTANCE if goal_direction_distance is None
            else goal_direction_distance)

    @staticmethod
    def reference_speed(distance, capture_speed=0.0):
        usable = max(float(distance) - config.GOAL_TOLERANCE_ASSUMED, 0.0)
        capture = float(capture_speed)
        return min(config.V_MAX, math.sqrt(capture * capture +
                                            2.0 * config.TERMINAL_DECEL_REF * usable))

    def update_mode(self, remaining_guide_distance, mission_time, position,
                    speed):
        self.current_mission_time = float(mission_time)
        if (not self.terminal_mode and
                remaining_guide_distance <= config.TERMINAL_TRIGGER_DISTANCE):
            self.terminal_mode = True
            self.entry_snapshot = {
                'mission_time': float(mission_time),
                'position': np.asarray(position, dtype='float64').copy(),
                'speed': float(speed),
                'remaining_guide_distance': float(remaining_guide_distance),
                'goal_distance': float(np.linalg.norm(self.goal - position)),
            }
            self.release_time = float(mission_time) + self.release_delay
        return self.terminal_mode

    def terminal_direction(self, position, guide_direction,
                           remaining_guide_distance, on_last_segment):
        goal_delta = self.goal - position
        norm = float(np.linalg.norm(goal_delta))
        if ((on_last_segment or remaining_guide_distance <=
             self.goal_direction_distance) and norm > 1.0e-12):
            return goal_delta / norm
        return np.asarray(guide_direction, dtype='float64')

    def speed_candidates(self, reference_speed, current_parallel_speed):
        delta = config.TERMINAL_SPEED_DELTA
        values = np.array([
            0.0, 0.5 * reference_speed, 0.75 * reference_speed,
            reference_speed, reference_speed - delta, reference_speed + delta,
            max(current_parallel_speed, 0.0), self.capture_speed], dtype='float64')
        return np.unique(np.clip(values, 0.0, config.V_MAX))

    @staticmethod
    def _feasibility(positions, velocities, accelerations, eps):
        max_speed = np.max(np.linalg.norm(velocities, axis=2), axis=1)
        max_xz = np.max(np.linalg.norm(accelerations[:, :, [0, 2]], axis=2), axis=1)
        max_ay = np.max(accelerations[:, :, 1], axis=1)
        min_ay = np.min(accelerations[:, :, 1], axis=1)
        speed_ok = max_speed <= config.V_MAX + eps
        altitude_ok = np.all(
            (positions[:, :, 1] >= config.Y_MIN_ASSUMED - eps) &
            (positions[:, :, 1] <= config.Y_MAX + eps), axis=1)
        xz_ok = max_xz <= config.A_XZ_MAX + eps
        up_ok = max_ay <= config.A_Y_UP_MAX + eps
        down_ok = min_ay >= -config.A_Y_DOWN_MAX - eps
        return (speed_ok, altitude_ok, xz_ok, up_ok, down_ok,
                speed_ok & altitude_ok & xz_ok & up_ok & down_ok,
                max_speed, max_xz, max_ay, min_ay)

    def generate(self, position, velocity, acceleration, guide_direction,
                 remaining_guide_distance):
        start = time.perf_counter()
        position = np.asarray(position, dtype='float64')
        velocity = np.asarray(velocity, dtype='float64')
        acceleration = np.asarray(acceleration, dtype='float64')
        direction = np.asarray(guide_direction, dtype='float64')
        direction = direction / np.linalg.norm(direction)
        _, _, lateral_direction = self.generator._directions(direction, velocity)
        goal_distance = float(np.linalg.norm(self.goal - position))
        effective_capture = 0.0 if self.legacy_envelope else self.capture_speed
        reference = self.reference_speed(goal_distance, effective_capture)
        if (self.release_time is not None and
                self.current_mission_time < self.release_time and
                self.wait_reference_speed is not None):
            reference = min(reference, self.wait_reference_speed)
        current_parallel = max(float(np.dot(velocity, direction)), 0.0)
        speeds = self.speed_candidates(reference, current_parallel)
        scale = float(np.clip(goal_distance / config.TERMINAL_TRIGGER_DISTANCE,
                              0.0, 1.0))
        lateral_values = self.generator.lateral_offset_values * scale
        vertical_values = self.generator.vertical_offset_values * scale
        lateral, vertical, terminal_speed = np.meshgrid(
            lateral_values, vertical_values, speeds, indexing='ij')
        lateral, vertical, terminal_speed = (lateral.ravel(), vertical.ravel(),
                                              terminal_speed.ravel())
        nominal = 0.5 * (current_parallel + terminal_speed) * self.generator.horizon
        # A non-exact endpoint may extend only one arrival-tolerance radius
        # past the guide goal.  Capping exactly at the goal causes a fixed-T
        # receding controller to postpone arrival asymptotically every cycle.
        fly_through = (0.0 if self.legacy_envelope else
                       self.capture_speed * self.generator.horizon)
        forward_limit = (max(float(remaining_guide_distance), 0.0) +
                         config.GOAL_TOLERANCE_ASSUMED + fly_through)
        forward = np.minimum(nominal, forward_limit)
        terminal_positions = (position + forward[:, None] * direction +
                              lateral[:, None] * lateral_direction +
                              vertical[:, None] * np.array([0.0, 1.0, 0.0]))
        terminal_velocities = terminal_speed[:, None] * direction

        exact_values = [0.0, 0.5 * reference, min(reference, 10.0)]
        if not self.legacy_envelope:
            exact_values.append(self.capture_speed)
        exact_speeds = np.unique(np.asarray(exact_values, dtype='float64'))
        terminal_positions = np.vstack(
            (terminal_positions, np.tile(self.goal, (len(exact_speeds), 1))))
        terminal_velocities = np.vstack(
            (terminal_velocities, exact_speeds[:, None] * direction))
        lateral = np.r_[lateral, np.zeros(len(exact_speeds))]
        vertical = np.r_[vertical, np.zeros(len(exact_speeds))]
        terminal_speed = np.r_[terminal_speed, exact_speeds]
        exact_mask = np.zeros(len(terminal_speed), dtype=bool)
        exact_mask[-len(exact_speeds):] = True
        capture_mask = np.zeros(len(terminal_speed), dtype=bool)
        if not self.legacy_envelope:
            approach_delta = self.goal - position
            approach_norm = float(np.linalg.norm(approach_delta))
            approach = (direction if approach_norm <= 1.0e-12 else
                        approach_delta / approach_norm)
            capture_point = self.goal - config.GOAL_TOLERANCE_ASSUMED * approach
            capture_speeds = np.unique(np.asarray(
                [self.capture_speed, reference], dtype='float64'))
            count = len(capture_speeds)
            terminal_positions = np.vstack(
                (terminal_positions, np.tile(capture_point, (count, 1))))
            terminal_velocities = np.vstack(
                (terminal_velocities, capture_speeds[:, None] * approach))
            lateral = np.r_[lateral, np.zeros(count)]
            vertical = np.r_[vertical, np.zeros(count)]
            terminal_speed = np.r_[terminal_speed, capture_speeds]
            exact_mask = np.r_[exact_mask, np.zeros(count, dtype=bool)]
            capture_mask = np.r_[capture_mask, np.ones(count, dtype=bool)]
        candidate_types = np.full(len(terminal_speed), 'normal', dtype='<U16')
        candidate_types[exact_mask] = 'exact_goal'
        candidate_types[capture_mask] = 'capture_boundary'
        terminal_acceleration = np.zeros_like(terminal_positions)
        coefficients = closed_form_quintic_coefficients(
            position, velocity, acceleration, terminal_positions,
            terminal_velocities, terminal_acceleration,
            self.generator.horizon)
        positions, velocities, accelerations = evaluate_quintics(
            coefficients, self.generator.times)
        values = self._feasibility(positions, velocities, accelerations,
                                   self.generator.constraint_epsilon)
        batch = PrimitiveBatch(
            self.generator.times.copy(), positions, velocities, accelerations,
            coefficients, terminal_positions, terminal_velocities,
            terminal_acceleration, lateral, vertical, terminal_speed,
            values[0], values[1], values[2], values[3], values[4], values[5],
            values[6], values[7], values[8], values[9])
        return TerminalPrimitiveContext(batch, exact_mask, capture_mask,
                                        candidate_types, reference, scale,
                                        direction, time.perf_counter() - start)

    def rank(self, dynamic_selection, context, goal_distance):
        start = time.perf_counter()
        endpoint_distance = np.linalg.norm(
            context.batch.terminal_positions - self.goal, axis=1)
        execution_index = int(np.argmin(np.abs(
            context.batch.times - config.EXECUTION_HORIZON)))
        prefix_distance = np.linalg.norm(
            context.batch.positions[:, execution_index] - self.goal, axis=1)
        prefix_weight = config.TERMINAL_EXECUTED_PREFIX_GOAL_WEIGHT
        goal_cost = ((endpoint_distance + prefix_weight * prefix_distance) /
                     ((1.0 + prefix_weight) *
                      config.TERMINAL_TRIGGER_DISTANCE))
        velocity_cost = (np.abs(context.batch.terminal_speeds -
                                context.reference_speed) / config.V_MAX)
        alpha = float(np.clip(1.0 - goal_distance /
                              config.TERMINAL_TRIGGER_DISTANCE, 0.0, 1.0))
        total = dynamic_selection.total_cost + alpha * (
            config.TERMINAL_COST_WEIGHT_GOAL * goal_cost +
            config.TERMINAL_COST_WEIGHT_VREF * velocity_cost)
        valid = np.flatnonzero(dynamic_selection.final_valid_mask)
        order = np.lexsort((valid, total[valid]))
        ranked = valid[order]
        best = int(ranked[0]) if len(ranked) else None
        return TerminalRanking(goal_cost, velocity_cost, total, ranked, best,
                               alpha, time.perf_counter() - start)
