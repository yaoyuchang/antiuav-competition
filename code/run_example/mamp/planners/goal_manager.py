"""Discrete base-goal switching and deterministic small dynamic-goal updates."""

from dataclasses import dataclass
import math
import time

import numpy as np

from ..configs import subject3_config as config


class IndividualSlantRangeTrigger(object):
    """Engineering interpretation: Euclidean range from an explicit origin."""

    def __init__(self, threshold=None, reference_origin=None):
        self.threshold = float(config.GOAL_SWITCH_SLANT_RANGE if threshold is None
                               else threshold)
        self.reference_origin = np.asarray(
            config.GOAL_SWITCH_REFERENCE_ORIGIN if reference_origin is None
            else reference_origin, dtype='float64')

    def should_switch_goal(self, position, mission_time=None):
        del mission_time
        return bool(np.linalg.norm(np.asarray(position) - self.reference_origin) >=
                    self.threshold)


class DynamicGoalModel(object):
    """Configurable DEMO sinusoid plus reproducible piecewise random error."""

    def __init__(self, sinusoid_enabled=False, random_enabled=False, seed=None,
                 uav_index=0, sinusoid_amplitude=None, sinusoid_frequency=None,
                 sinusoid_phase=None, random_amplitude=None,
                 random_update_frequency=None):
        self.sinusoid_enabled = bool(sinusoid_enabled)
        self.random_enabled = bool(random_enabled)
        self.sinusoid_amplitude = np.asarray(
            config.GOAL_SINUSOID_AMPLITUDE if sinusoid_amplitude is None
            else sinusoid_amplitude, dtype='float64')
        self.sinusoid_frequency = float(
            config.GOAL_SINUSOID_FREQUENCY if sinusoid_frequency is None
            else sinusoid_frequency)
        self.sinusoid_phase = np.asarray(
            config.GOAL_SINUSOID_PHASE if sinusoid_phase is None
            else sinusoid_phase, dtype='float64')
        self.random_amplitude = np.asarray(
            config.GOAL_RANDOM_AMPLITUDE if random_amplitude is None
            else random_amplitude, dtype='float64')
        self.random_update_frequency = float(
            config.GOAL_RANDOM_UPDATE_FREQUENCY if random_update_frequency is None
            else random_update_frequency)
        base_seed = config.GOAL_RANDOM_SEED if seed is None else int(seed)
        self.rng = np.random.RandomState(base_seed + 10007 * int(uav_index))
        self._random_values = {}

    def _random_error(self, mission_time):
        if not self.random_enabled or self.random_update_frequency <= 0.0:
            return np.zeros(3)
        bucket = int(math.floor(max(float(mission_time), 0.0) *
                                self.random_update_frequency + 1.0e-12))
        while len(self._random_values) <= bucket:
            index = len(self._random_values)
            self._random_values[index] = self.rng.uniform(
                -self.random_amplitude, self.random_amplitude)
        return self._random_values[bucket].copy()

    def observe(self, base_goal, mission_time):
        base = np.asarray(base_goal, dtype='float64')
        offset = np.zeros(3)
        if self.sinusoid_enabled:
            angle = 2.0 * math.pi * self.sinusoid_frequency * float(mission_time)
            offset += self.sinusoid_amplitude * np.sin(angle + self.sinusoid_phase)
        offset += self._random_error(mission_time)
        return base + offset


@dataclass
class GoalUpdate:
    switched: bool
    goal_version: int
    base_goal: np.ndarray
    observed_goal: np.ndarray
    filtered_goal: np.ndarray
    update_time: float


class GoalManager(object):
    def __init__(self, initial_base_goal, switched_base_goal,
                 trigger_policy=None, dynamic_model=None, filter_alpha=None,
                 deadband=None, arrival_goal_mode=None, switch_enabled=True):
        self.initial_base_goal = np.asarray(initial_base_goal, dtype='float64').copy()
        self.switched_base_goal = np.asarray(switched_base_goal, dtype='float64').copy()
        self.active_base_goal = self.initial_base_goal.copy()
        self.trigger_policy = trigger_policy or IndividualSlantRangeTrigger()
        self.dynamic_model = dynamic_model or DynamicGoalModel()
        self.filter_alpha = float(config.GOAL_FILTER_ALPHA if filter_alpha is None
                                  else filter_alpha)
        self.deadband = float(config.GOAL_DEADBAND if deadband is None else deadband)
        self.arrival_goal_mode = (config.ARRIVAL_GOAL_MODE if arrival_goal_mode is None
                                  else str(arrival_goal_mode))
        if not 0.0 <= self.filter_alpha <= 1.0:
            raise ValueError('filter_alpha must be in [0, 1]')
        if self.arrival_goal_mode not in ('observed', 'filtered'):
            raise ValueError('arrival_goal_mode must be observed or filtered')
        self.switch_enabled = bool(switch_enabled)
        self.goal_switched = False
        self.goal_version = 0
        self.switch_time = None
        self.switch_position = None
        self.old_goal = None
        self.new_goal = None
        self.observed_dynamic_goal = self.initial_base_goal.copy()
        self.filtered_dynamic_goal = self.initial_base_goal.copy()
        self.planning_goal = self.initial_base_goal.copy()
        self.history = []

    @property
    def arrival_goal(self):
        return (self.observed_dynamic_goal if self.arrival_goal_mode == 'observed'
                else self.filtered_dynamic_goal)

    def true_goal_at(self, mission_time):
        """Return the unfiltered model goal for independent evaluation."""
        return self.dynamic_model.observe(self.active_base_goal, mission_time)

    def update(self, position, mission_time):
        start = time.perf_counter()
        switched = False
        if (self.switch_enabled and not self.goal_switched and
                self.trigger_policy.should_switch_goal(position, mission_time)):
            switched = True
            self.goal_switched = True
            self.goal_version = 1
            self.switch_time = float(mission_time)
            self.switch_position = np.asarray(position, dtype='float64').copy()
            self.old_goal = self.active_base_goal.copy()
            self.active_base_goal = self.switched_base_goal.copy()
            self.new_goal = self.active_base_goal.copy()
        observed = self.dynamic_model.observe(self.active_base_goal, mission_time)
        if switched:
            filtered = observed.copy()
        else:
            delta = observed - self.filtered_dynamic_goal
            filtered = (self.filtered_dynamic_goal.copy() if np.linalg.norm(delta) < self.deadband
                        else (1.0 - self.filter_alpha) * self.filtered_dynamic_goal +
                        self.filter_alpha * observed)
        self.observed_dynamic_goal = observed.copy()
        self.filtered_dynamic_goal = filtered.copy()
        self.planning_goal = filtered.copy()
        elapsed = time.perf_counter() - start
        update = GoalUpdate(switched, self.goal_version,
                            self.active_base_goal.copy(), observed.copy(),
                            filtered.copy(), elapsed)
        self.history.append({'mission_time': float(mission_time),
                             'goal_version': self.goal_version,
                             'base_goal': self.active_base_goal.copy(),
                             'observed_goal': observed.copy(),
                             'filtered_goal': filtered.copy(),
                             'goal_update_time': elapsed})
        return update
