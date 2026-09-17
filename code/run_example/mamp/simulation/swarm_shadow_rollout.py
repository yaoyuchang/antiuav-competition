"""One-step analytic shadow rollout isolated from formal planner state."""

from dataclasses import dataclass
import copy

import numpy as np

from ..configs import subject3_config as config
from ..planners.local_joint_coordinator import LocalJointCoordinator
from ..planners.quintic_primitive import evaluate_quintics
from ..planners.swarm_base_batch_planner import SwarmBaseBatchPlanner


@dataclass
class ShadowFeasibilityResult:
    success: bool
    states: tuple
    future_base_valid_counts: np.ndarray
    future_selected_indices: np.ndarray
    failure_snapshot: object


@dataclass
class RollingFailureContext:
    cycle: int
    planning_time: float
    plans: tuple
    states: tuple
    coordination: object


def analytic_shadow_states(plans, selected_indices, execution_horizon=None):
    horizon = float(config.EXECUTION_HORIZON if execution_horizon is None
                    else execution_horizon)
    times = np.asarray([horizon], dtype='float64')
    states = []
    for plan, index in zip(plans, selected_indices):
        p, v, a = evaluate_quintics(
            plan.primitive_batch.coefficients[int(index):int(index) + 1], times)
        states.append((p[0, 0].copy(), v[0, 0].copy(), a[0, 0].copy()))
    return tuple(states)


def one_step_future_feasibility(planners, plans, selected_indices,
                                planning_time, execution_horizon=None,
                                uav_ids=None):
    """Regenerate Phase3--5 on deep-copied planners at the exact shadow state."""
    horizon = float(config.EXECUTION_HORIZON if execution_horizon is None
                    else execution_horizon)
    states = analytic_shadow_states(plans, selected_indices, horizon)
    ids = (np.arange(len(plans), dtype='int64') if uav_ids is None else
           np.asarray(uav_ids, dtype='int64'))
    local_states = tuple(states[int(index)] for index in ids)
    shadow_planners = copy.deepcopy(tuple(planners[int(index)] for index in ids))
    future_plans, _, _ = SwarmBaseBatchPlanner().plan(
        shadow_planners, local_states, float(planning_time) + horizon)
    future = LocalJointCoordinator(top_k=5).coordinate(
        future_plans, local_states, float(planning_time) + horizon)
    counts = np.asarray([len(item.dynamic_selection.ranked_indices)
                         for item in future_plans], dtype='int64')
    return ShadowFeasibilityResult(
        future.success, states, counts, future.selected_indices.copy(),
        future.failure_snapshot)


def replay_until_failure(planners, starts, coordinator, max_cycles=500):
    """Diagnostic replay returning the first failed cycle without state repair."""
    positions = np.asarray(starts, dtype='float64').copy()
    velocities = np.zeros_like(positions)
    accelerations = np.zeros_like(positions)
    batcher = SwarmBaseBatchPlanner()
    for cycle in range(int(max_cycles)):
        planning_time = cycle * config.EXECUTION_HORIZON
        states = tuple((positions[i], velocities[i], accelerations[i])
                       for i in range(len(positions)))
        plans, _, _ = batcher.plan(planners, states, planning_time)
        result = coordinator.coordinate(plans, states, planning_time)
        if not result.success:
            return RollingFailureContext(cycle, planning_time, tuple(plans),
                                         states, result)
        next_states = analytic_shadow_states(plans, result.selected_indices)
        positions = np.asarray([item[0] for item in next_states])
        velocities = np.asarray([item[1] for item in next_states])
        accelerations = np.asarray([item[2] for item in next_states])
    return None
