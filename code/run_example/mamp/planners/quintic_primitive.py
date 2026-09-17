"""Vectorized quintic motion primitive generation without collision checks."""

from dataclasses import dataclass
import math

import numpy as np

from ..configs import subject3_config as config


_POLYNOMIAL_BASIS_CACHE = {}


def polynomial_bases(times):
    """Return immutable shared quintic position/velocity/acceleration bases."""
    values = np.asarray(times, dtype='float64')
    key = (values.shape, values.tobytes())
    cached = _POLYNOMIAL_BASIS_CACHE.get(key)
    if cached is not None:
        return cached
    powers = np.arange(6, dtype='int64')[:, None]
    position_basis = values[None, :] ** powers
    velocity_basis = np.zeros_like(position_basis)
    acceleration_basis = np.zeros_like(position_basis)
    velocity_basis[1:] = powers[1:] * values[None, :] ** (powers[1:] - 1)
    acceleration_basis[2:] = (powers[2:] * (powers[2:] - 1) *
                              values[None, :] ** (powers[2:] - 2))
    for basis in (position_basis, velocity_basis, acceleration_basis):
        basis.flags.writeable = False
    cached = (position_basis, velocity_basis, acceleration_basis)
    _POLYNOMIAL_BASIS_CACHE[key] = cached
    return cached


@dataclass
class PrimitiveBatch:
    """One structure of arrays for all candidates in a planning cycle."""

    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    coefficients: np.ndarray
    terminal_positions: np.ndarray
    terminal_velocities: np.ndarray
    terminal_accelerations: np.ndarray
    lateral_offsets: np.ndarray
    vertical_offsets: np.ndarray
    terminal_speeds: np.ndarray
    speed_ok: np.ndarray
    altitude_ok: np.ndarray
    xz_acc_ok: np.ndarray
    y_up_ok: np.ndarray
    y_down_ok: np.ndarray
    feasible_mask: np.ndarray
    max_speed: np.ndarray
    max_xz_acc: np.ndarray
    max_ay: np.ndarray
    min_ay: np.ndarray


def closed_form_quintic_coefficients(p0, v0, a0, pf, vf, af, horizon):
    """Return batched coefficients satisfying six endpoint conditions exactly."""
    p0 = np.asarray(p0, dtype='float64')
    v0 = np.asarray(v0, dtype='float64')
    a0 = np.asarray(a0, dtype='float64')
    pf = np.asarray(pf, dtype='float64')
    vf = np.asarray(vf, dtype='float64')
    af = np.asarray(af, dtype='float64')
    if p0.shape != (3,) or v0.shape != (3,) or a0.shape != (3,):
        raise ValueError('p0, v0 and a0 must have shape (3,)')
    if pf.ndim != 2 or pf.shape[1] != 3 or vf.shape != pf.shape or af.shape != pf.shape:
        raise ValueError('pf, vf and af must have matching shape (Np, 3)')
    horizon = float(horizon)
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError('horizon must be finite and positive')

    delta_p = pf - (p0 + v0 * horizon + 0.5 * a0 * horizon ** 2)
    delta_v = vf - (v0 + a0 * horizon)
    delta_a = af - a0
    coefficients = np.empty((len(pf), 6, 3), dtype='float64')
    coefficients[:, 0, :] = p0
    coefficients[:, 1, :] = v0
    coefficients[:, 2, :] = 0.5 * a0
    coefficients[:, 3, :] = (10.0 * delta_p / horizon ** 3 -
                              4.0 * delta_v / horizon ** 2 +
                              0.5 * delta_a / horizon)
    coefficients[:, 4, :] = (-15.0 * delta_p / horizon ** 4 +
                              7.0 * delta_v / horizon ** 3 -
                              delta_a / horizon ** 2)
    coefficients[:, 5, :] = (6.0 * delta_p / horizon ** 5 -
                              3.0 * delta_v / horizon ** 4 +
                              0.5 * delta_a / horizon ** 3)
    return coefficients


def evaluate_quintics(coefficients, times):
    """Evaluate every primitive at every shared sample time using matrix products."""
    coefficients = np.asarray(coefficients, dtype='float64')
    times = np.asarray(times, dtype='float64')
    if coefficients.ndim != 3 or coefficients.shape[1:] != (6, 3):
        raise ValueError('coefficients must have shape (Np, 6, 3)')
    if times.ndim != 1:
        raise ValueError('times must have shape (Nt,)')
    position_basis, velocity_basis, acceleration_basis = polynomial_bases(times)
    candidate_count = len(coefficients)
    coordinate_coefficients = coefficients.transpose(0, 2, 1).reshape(-1, 6)
    def evaluate_basis(basis):
        return coordinate_coefficients.dot(basis).reshape(
            candidate_count, 3, -1).transpose(0, 2, 1)
    positions = evaluate_basis(position_basis)
    velocities = evaluate_basis(velocity_basis)
    accelerations = evaluate_basis(acceleration_basis)
    return positions, velocities, accelerations


class QuinticPrimitiveGenerator(object):
    """Generate parameterized single-UAV primitives as one vectorized batch."""

    def __init__(self, horizon=None, dt=None, lateral_offsets=None,
                 vertical_offsets=None, delta_speeds=None, v_end_min=None,
                 high_speed_target=None, direction_epsilon=None,
                 constraint_epsilon=None):
        self.horizon = float(config.PRIMITIVE_HORIZON if horizon is None else horizon)
        self.dt = float(config.PRIMITIVE_DT if dt is None else dt)
        if self.horizon <= 0.0 or self.dt <= 0.0:
            raise ValueError('horizon and dt must be positive')
        self.lateral_offset_values = np.asarray(
            config.PRIMITIVE_LATERAL_OFFSETS if lateral_offsets is None
            else lateral_offsets, dtype='float64')
        self.vertical_offset_values = np.asarray(
            config.PRIMITIVE_VERTICAL_OFFSETS if vertical_offsets is None
            else vertical_offsets, dtype='float64')
        self.delta_speeds = np.asarray(
            config.PRIMITIVE_DELTA_SPEED_CANDIDATES if delta_speeds is None
            else delta_speeds, dtype='float64')
        self.v_end_min = float(config.PRIMITIVE_V_END_MIN if v_end_min is None
                               else v_end_min)
        self.high_speed_target = float(
            config.PRIMITIVE_HIGH_SPEED_TARGET_RATIO * config.V_MAX
            if high_speed_target is None else high_speed_target)
        self.direction_epsilon = float(
            config.PRIMITIVE_DIRECTION_EPS if direction_epsilon is None
            else direction_epsilon)
        self.constraint_epsilon = float(
            config.PRIMITIVE_CONSTRAINT_EPS if constraint_epsilon is None
            else constraint_epsilon)
        intervals = int(math.ceil(self.horizon / self.dt))
        self.times = np.linspace(0.0, self.horizon, intervals + 1)
        self.previous_horizontal_forward = None

    @staticmethod
    def _state_vector(value, name):
        result = np.asarray(value, dtype='float64')
        if result.shape != (3,) or not np.all(np.isfinite(result)):
            raise ValueError('{} must be a finite vector with shape (3,)'.format(name))
        return result

    def _directions(self, global_direction, velocity):
        direction = self._state_vector(global_direction, 'global_direction')
        norm = float(np.linalg.norm(direction))
        if norm <= self.direction_epsilon:
            raise ValueError('global_direction must be nonzero')
        direction = direction / norm
        horizontal = np.array([direction[0], 0.0, direction[2]])
        horizontal_norm = float(np.linalg.norm(horizontal))
        if horizontal_norm > self.direction_epsilon:
            forward = horizontal / horizontal_norm
            self.previous_horizontal_forward = forward.copy()
        else:
            velocity_horizontal = np.array([velocity[0], 0.0, velocity[2]])
            velocity_norm = float(np.linalg.norm(velocity_horizontal))
            if velocity_norm > self.direction_epsilon:
                forward = velocity_horizontal / velocity_norm
            elif self.previous_horizontal_forward is not None:
                forward = self.previous_horizontal_forward.copy()
            else:
                forward = np.array([1.0, 0.0, 0.0])
        lateral = np.array([-forward[2], 0.0, forward[0]])
        return direction, forward, lateral

    def terminal_speed_candidates(self, v_parallel):
        base_speed = max(float(v_parallel), 0.0)
        speeds = np.clip(base_speed + self.delta_speeds,
                         self.v_end_min, config.V_MAX)
        speeds = np.append(speeds, np.clip(self.high_speed_target,
                                           self.v_end_min, config.V_MAX))
        return np.unique(speeds)

    def generate(self, p0, v0, a0, global_direction, max_forward_distance=None):
        p0 = self._state_vector(p0, 'p0')
        v0 = self._state_vector(v0, 'v0')
        a0 = self._state_vector(a0, 'a0')
        direction, _, lateral_direction = self._directions(global_direction, v0)
        v_parallel = max(float(np.dot(v0, direction)), 0.0)
        speed_values = self.terminal_speed_candidates(v_parallel)

        lateral, vertical, terminal_speed = np.meshgrid(
            self.lateral_offset_values, self.vertical_offset_values,
            speed_values, indexing='ij')
        lateral = lateral.ravel()
        vertical = vertical.ravel()
        terminal_speed = terminal_speed.ravel()
        forward_distance = 0.5 * (v_parallel + terminal_speed) * self.horizon
        if max_forward_distance is not None:
            maximum = float(max_forward_distance)
            if not math.isfinite(maximum) or maximum < 0.0:
                raise ValueError('max_forward_distance must be finite and nonnegative')
            forward_distance = np.minimum(forward_distance, maximum)
        terminal_positions = (p0[None, :] + forward_distance[:, None] * direction +
                              lateral[:, None] * lateral_direction +
                              vertical[:, None] * np.array([0.0, 1.0, 0.0]))
        terminal_velocities = terminal_speed[:, None] * direction
        terminal_accelerations = np.zeros_like(terminal_positions)
        coefficients = closed_form_quintic_coefficients(
            p0, v0, a0, terminal_positions, terminal_velocities,
            terminal_accelerations, self.horizon)
        positions, velocities, accelerations = evaluate_quintics(
            coefficients, self.times)

        max_speed = np.max(np.linalg.norm(velocities, axis=2), axis=1)
        max_xz_acc = np.max(np.linalg.norm(accelerations[:, :, [0, 2]], axis=2), axis=1)
        max_ay = np.max(accelerations[:, :, 1], axis=1)
        min_ay = np.min(accelerations[:, :, 1], axis=1)
        eps = self.constraint_epsilon
        speed_ok = max_speed <= config.V_MAX + eps
        altitude_ok = np.all(
            (positions[:, :, 1] >= config.Y_MIN_ASSUMED - eps) &
            (positions[:, :, 1] <= config.Y_MAX + eps), axis=1)
        xz_acc_ok = max_xz_acc <= config.A_XZ_MAX + eps
        y_up_ok = max_ay <= config.A_Y_UP_MAX + eps
        y_down_ok = min_ay >= -config.A_Y_DOWN_MAX - eps
        feasible_mask = speed_ok & altitude_ok & xz_acc_ok & y_up_ok & y_down_ok
        return PrimitiveBatch(
            self.times.copy(), positions, velocities, accelerations, coefficients,
            terminal_positions, terminal_velocities, terminal_accelerations,
            lateral, vertical, terminal_speed, speed_ok, altitude_ok, xz_acc_ok,
            y_up_ok, y_down_ok, feasible_mask, max_speed, max_xz_acc, max_ay,
            min_ay)
