"""Pooled Phase3--5 computation preserving independent single-UAV semantics."""

from dataclasses import dataclass
import math
import time

import numpy as np

from ..collision.dynamic_primitive_collision import distance_to_predicted_obstacle
from ..collision.static_primitive_collision import FineStaticValidation
from ..configs import subject3_config as config
from .dynamic_primitive_selector import DynamicPrimitiveSelection
from .primitive_selector import StaticPrimitiveSelection
from .quintic_primitive import evaluate_quintics


@dataclass
class SwarmCandidatePool:
    coefficients: np.ndarray
    owner_uav: np.ndarray
    local_candidate_index: np.ndarray
    uav_offsets: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray


@dataclass
class BatchBasePlan:
    success: bool
    reason: str
    planning_time: float
    primitive_batch: object
    static_selection: object
    dynamic_selection: object
    guide_direction: np.ndarray
    guide_target: np.ndarray
    mode: str
    failure_snapshot: object = None


@dataclass
class BatchPlanningTiming:
    phase3_generation: float
    pool_build: float
    fine_validation: float
    static_collision: float
    phase4_cost_ranking: float
    dynamic_prediction: float
    dynamic_collision: float
    phase5_cost_ranking: float
    total: float
    trajectory_bytes: int
    peak_working_bytes_estimate: int
    static_obstacles_considered: int = 0
    static_exact_evaluations: int = 0
    static_obstacles_skipped: int = 0
    static_candidate_obstacle_pairs: int = 0


def pooled_static_clearance(positions, obstacles):
    """Exact minimum clearance with a conservative bounding-sphere broad phase."""
    candidate_count, sample_count = positions.shape[:2]
    minimum = np.full(candidate_count, np.inf)
    closest_id = np.full(candidate_count, -1, dtype='int64')
    lower = np.min(positions, axis=1)
    upper = np.max(positions, axis=1)
    bounds = []
    for obstacle in obstacles:
        center = obstacle.pos_global_frame
        gap = np.maximum(np.maximum(lower - center, center - upper), 0.0)
        sphere_lower = np.maximum(np.linalg.norm(gap, axis=1) - obstacle.radius, 0.0)
        bounds.append((float(np.median(sphere_lower)), obstacle, sphere_lower))
    bounds.sort(key=lambda item: item[0])
    exact_calls = skipped = pair_evaluations = peak_bytes = 0
    for _, obstacle, sphere_lower in bounds:
        active = np.flatnonzero(sphere_lower < minimum)
        if not len(active):
            skipped += 1
            continue
        distance = np.sqrt(obstacle.distance_sq_to_points(
            positions[active].reshape(-1, 3))).reshape(len(active), sample_count)
        exact_calls += 1
        pair_evaluations += len(active)
        peak_bytes = max(peak_bytes, distance.nbytes)
        obstacle_minimum = np.min(distance, axis=1)
        improved_local = obstacle_minimum < minimum[active]
        improved = active[improved_local]
        minimum[improved] = obstacle_minimum[improved_local]
        closest_id[improved] = int(obstacle.competition_id)
    statistics = {'obstacles_considered': len(obstacles),
                  'exact_evaluations': exact_calls,
                  'obstacles_skipped': skipped,
                  'candidate_obstacle_pairs': pair_evaluations,
                  'peak_distance_bytes': peak_bytes}
    return minimum, closest_id, statistics


class SwarmBaseBatchPlanner(object):
    def _generate(self, planner, state, planning_time):
        position, velocity, acceleration = [np.asarray(x, dtype='float64') for x in state]
        remaining = planner.guide_tracker.get_remaining_path_length(position)
        direction = planner.guide_tracker.get_turn_aware_direction(position, velocity)
        target = planner.guide_tracker.last_target.copy()
        terminal = (False if planner.terminal_policy is None else
                    planner.terminal_policy.update_mode(
                        remaining, planning_time, position, np.linalg.norm(velocity)))
        if terminal:
            on_last = planner.guide_tracker.current_segment_index >= len(
                planner.guide_tracker.waypoints) - 2
            direction = planner.terminal_policy.terminal_direction(
                position, direction, remaining, on_last)
            context = planner.terminal_policy.generate(
                position, velocity, acceleration, direction, remaining)
            batch = context.batch
        else:
            batch = planner.generator.generate(position, velocity, acceleration, direction)
        return batch, direction, target, 'TERMINAL' if terminal else 'CRUISE'

    def plan(self, planners, states, planning_time):
        total_start = time.perf_counter()
        generation_start = time.perf_counter()
        generated = [self._generate(planner, state, planning_time)
                     for planner, state in zip(planners, states)]
        generation_time = time.perf_counter() - generation_start

        pool_start = time.perf_counter()
        batches = [item[0] for item in generated]
        counts = np.asarray([len(batch.positions) for batch in batches], dtype='int64')
        offsets = np.r_[0, np.cumsum(counts)]
        coefficients = np.concatenate([batch.coefficients for batch in batches], axis=0)
        owner = np.repeat(np.arange(len(batches), dtype='int64'), counts)
        local = np.concatenate([np.arange(count, dtype='int64') for count in counts])
        pool_time = time.perf_counter() - pool_start

        checker = planners[0].static_selector.collision_checker
        validation_times = checker.validation_times
        validation_start = time.perf_counter()
        positions, velocities, accelerations = evaluate_quintics(
            coefficients, validation_times)
        speed = np.linalg.norm(velocities, axis=2)
        xz_acceleration = np.linalg.norm(accelerations[:, :, [0, 2]], axis=2)
        eps = checker.constraint_epsilon
        fine_speed = np.max(speed, axis=1) <= config.V_MAX + eps
        fine_altitude = np.all((positions[:, :, 1] >= config.Y_MIN_ASSUMED - eps) &
                               (positions[:, :, 1] <= config.Y_MAX + eps), axis=1)
        fine_xz = np.max(xz_acceleration, axis=1) <= config.A_XZ_MAX + eps
        fine_up = np.max(accelerations[:, :, 1], axis=1) <= config.A_Y_UP_MAX + eps
        fine_down = np.min(accelerations[:, :, 1], axis=1) >= -config.A_Y_DOWN_MAX - eps
        fine_dynamic = fine_speed & fine_altitude & fine_xz & fine_up & fine_down
        validation_time = time.perf_counter() - validation_start

        static_start = time.perf_counter()
        static_minimum, static_closest_id, static_statistics = pooled_static_clearance(
            positions, checker.static_obstacles)
        peak_distance_bytes = static_statistics['peak_distance_bytes']
        static_safe = static_minimum >= (config.OBS_SAFE_DISTANCE +
                                         checker.collision_guard - eps)
        static_time = time.perf_counter() - static_start

        phase4_start = time.perf_counter()
        start_positions = np.asarray([state[0] for state in states])[owner]
        guide_directions = np.asarray([item[1] for item in generated])[owner]
        terminal_positions = np.concatenate(
            [batch.terminal_positions for batch in batches], axis=0)
        lateral = np.concatenate([batch.lateral_offsets for batch in batches])
        vertical = np.concatenate([batch.vertical_offsets for batch in batches])
        progress = np.einsum('ij,ij->i', terminal_positions - start_positions,
                             guide_directions)
        progress_cost = -progress / config.PRIMITIVE_PROGRESS_SCALE
        offset_cost = .5 * (np.abs(lateral) / max(config.PRIMITIVE_LATERAL_SCALE, 1.e-12) +
                            np.abs(vertical) / max(config.PRIMITIVE_VERTICAL_SCALE, 1.e-12))
        cross = np.linalg.norm(np.cross(velocities, accelerations), axis=2)
        curvature = np.divide(cross, speed ** 3, out=np.zeros_like(cross),
                              where=speed > 1.e-8)
        curvature_integral = np.trapz(curvature ** 2 * speed, validation_times, axis=1)
        curvature_cost = curvature_integral / config.PRIMITIVE_CURVATURE_REFERENCE
        deficit = np.maximum(config.PRIMITIVE_PREFERRED_CLEARANCE - static_minimum, 0.)
        clearance_cost = (deficit / config.PRIMITIVE_CLEARANCE_SCALE) ** 2
        horizontal_ratio = xz_acceleration / config.A_XZ_MAX
        vertical_ratio = np.where(accelerations[:, :, 1] >= 0.,
            accelerations[:, :, 1] / config.A_Y_UP_MAX,
            -accelerations[:, :, 1] / config.A_Y_DOWN_MAX)
        effort = np.trapz(horizontal_ratio ** 2 + vertical_ratio ** 2,
                          validation_times, axis=1) / checker.horizon
        phase4_total = (config.PRIMITIVE_COST_WEIGHT_PROGRESS * progress_cost +
                        config.PRIMITIVE_COST_WEIGHT_OFFSET * offset_cost +
                        config.PRIMITIVE_COST_WEIGHT_CURVATURE * curvature_cost +
                        config.PRIMITIVE_COST_WEIGHT_CLEARANCE * clearance_cost +
                        config.PRIMITIVE_COST_WEIGHT_EFFORT_PROXY * effort)
        coarse = np.concatenate([batch.feasible_mask for batch in batches])
        phase4_valid = coarse & fine_dynamic & static_safe
        phase4_time = time.perf_counter() - phase4_start

        prediction_start = time.perf_counter()
        dynamic_checker = planners[0].dynamic_selector.collision_checker
        absolute_times = float(planning_time) + validation_times
        predictions = []
        max_translation = max_rotation = 0.
        for obstacle in dynamic_checker.dynamic_obstacles:
            centers, yaw = obstacle.predict_pose(absolute_times)
            yaw_rate = obstacle.predict_yaw_rate(absolute_times)
            translation = obstacle.translation_speed_bound()
            rotation = (float(np.max(np.abs(yaw_rate))) * math.sqrt(
                (obstacle.length / 2.) ** 2 + (obstacle.width / 2.) ** 2)
                if obstacle.shape in ('cube', 'rotated_cube') else 0.)
            max_translation, max_rotation = max(max_translation, translation), max(max_rotation, rotation)
            predictions.append((obstacle, centers, np.asarray(yaw)))
        prediction_time = time.perf_counter() - prediction_start
        dynamic_guard = .5 * (config.V_MAX + max_translation + max_rotation) * config.VALIDATION_DT

        dynamic_start = time.perf_counter()
        dynamic_safe = np.zeros(len(coefficients), dtype=bool)
        dynamic_minimum = np.full(len(coefficients), np.inf)
        closest_id = np.full(len(coefficients), -1, dtype='int64')
        time_of_min = np.full(len(coefficients), np.nan)
        valid_indices = np.flatnonzero(phase4_valid)
        if len(valid_indices):
            candidate_positions = positions[valid_indices]
            candidate_minimum = np.full(len(valid_indices), np.inf)
            candidate_obstacle = np.full(len(valid_indices), -1, dtype='int64')
            candidate_time = np.full(len(valid_indices), np.nan)
            candidate_lower = np.min(candidate_positions, axis=1)
            candidate_upper = np.max(candidate_positions, axis=1)
            prediction_bounds = []
            for obstacle, centers, yaw in predictions:
                center_lower, center_upper = np.min(centers, axis=0), np.max(centers, axis=0)
                gap = np.maximum(np.maximum(candidate_lower - center_upper,
                                            center_lower - candidate_upper), 0.)
                lower_bound = np.maximum(np.linalg.norm(gap, axis=1) -
                                         obstacle.radius, 0.)
                prediction_bounds.append((float(np.median(lower_bound)), obstacle,
                                          centers, yaw, lower_bound))
            prediction_bounds.sort(key=lambda item: item[0])
            for _, obstacle, centers, yaw, lower_bound in prediction_bounds:
                active = np.flatnonzero(lower_bound < candidate_minimum)
                if not len(active):
                    continue
                distance = distance_to_predicted_obstacle(
                    candidate_positions[active], obstacle, centers, yaw)
                time_indices = np.argmin(distance, axis=1)
                rows = np.arange(len(active))
                obstacle_minimum = distance[rows, time_indices]
                improved_local = obstacle_minimum < candidate_minimum[active]
                improved = active[improved_local]
                candidate_minimum[improved] = obstacle_minimum[improved_local]
                candidate_obstacle[improved] = int(obstacle.competition_id)
                candidate_time[improved] = validation_times[
                    time_indices[improved_local]]
            dynamic_minimum[valid_indices] = candidate_minimum
            closest_id[valid_indices] = candidate_obstacle
            time_of_min[valid_indices] = candidate_time
            dynamic_safe[valid_indices] = candidate_minimum >= (
                config.OBS_SAFE_DISTANCE + dynamic_guard - dynamic_checker.constraint_epsilon)
        dynamic_time = time.perf_counter() - dynamic_start

        phase5_start = time.perf_counter()
        dynamic_deficit = np.maximum(config.DYNAMIC_PREFERRED_CLEARANCE -
                                     dynamic_minimum, 0.)
        dynamic_cost = (dynamic_deficit / config.DYNAMIC_CLEARANCE_SCALE) ** 2
        phase5_total = phase4_total + config.DYNAMIC_CLEARANCE_COST_WEIGHT * dynamic_cost
        final_valid = phase4_valid & dynamic_safe
        plans = []
        for uav_index, (batch, item) in enumerate(zip(batches, generated)):
            lo, hi = offsets[uav_index], offsets[uav_index + 1]
            sl = slice(lo, hi)
            validation = FineStaticValidation(
                validation_times, positions[sl], velocities[sl], accelerations[sl],
                fine_speed[sl], fine_altitude[sl], fine_xz[sl], fine_up[sl],
                fine_down[sl], fine_dynamic[sl], static_minimum[sl],
                static_safe[sl], validation_time, static_time)
            valid_local = np.flatnonzero(phase4_valid[sl])
            phase4_order = np.lexsort((valid_local, phase4_total[sl][valid_local]))
            phase4_ranked = valid_local[phase4_order]
            static_selection = StaticPrimitiveSelection(
                bool(len(phase4_ranked)), 'success' if len(phase4_ranked) else
                'no statically safe primitive', batch, validation,
                fine_dynamic[sl], static_safe[sl], phase4_valid[sl],
                static_minimum[sl], progress[sl], progress_cost[sl],
                curvature_cost[sl], clearance_cost[sl], offset_cost[sl], effort[sl],
                phase4_total[sl], phase4_ranked,
                int(phase4_ranked[0]) if len(phase4_ranked) else None,
                validation_times, validation_time, static_time, phase4_time, 0.,
                validation_time + static_time + phase4_time)
            final_local = np.flatnonzero(final_valid[sl])
            phase5_order = np.lexsort((final_local, phase5_total[sl][final_local]))
            phase5_ranked = final_local[phase5_order]
            dynamic_selection = DynamicPrimitiveSelection(
                bool(len(phase5_ranked)), 'success' if len(phase5_ranked) else
                'no dynamically safe primitive', static_selection, float(planning_time),
                absolute_times, dynamic_safe[sl], final_valid[sl],
                dynamic_minimum[sl], closest_id[sl], time_of_min[sl],
                dynamic_cost[sl], phase5_total[sl], phase5_ranked,
                int(phase5_ranked[0]) if len(phase5_ranked) else None,
                dynamic_guard, max_translation, max_rotation, prediction_time,
                dynamic_time, 0., 0., prediction_time + dynamic_time)
            success = bool(len(phase5_ranked))
            failure_snapshot = None
            if not success:
                state = states[uav_index]
                coarse_count = int(np.count_nonzero(coarse[sl]))
                fine_count = int(np.count_nonzero(fine_dynamic[sl]))
                phase4_count = int(np.count_nonzero(phase4_valid[sl]))
                nearest_static_local = int(np.argmin(static_minimum[sl]))
                nearest_dynamic_local = int(np.argmin(dynamic_minimum[sl]))
                failure_snapshot = {
                    'planning_time': float(planning_time),
                    'uav_offset': int(uav_index),
                    'mode': item[3],
                    'position': np.asarray(state[0]).copy(),
                    'velocity': np.asarray(state[1]).copy(),
                    'acceleration': np.asarray(state[2]).copy(),
                    'candidate_count': int(hi - lo),
                    'phase3_feasible_count': coarse_count,
                    'fine_dynamic_count': fine_count,
                    'phase4_valid_count': phase4_count,
                    'phase5_valid_count': int(len(phase5_ranked)),
                    'nearest_static_obstacle_id': int(
                        static_closest_id[lo + nearest_static_local]),
                    'minimum_static_clearance': float(
                        static_minimum[lo + nearest_static_local]),
                    'nearest_dynamic_obstacle_id': int(
                        closest_id[lo + nearest_dynamic_local]),
                    'minimum_dynamic_clearance': float(
                        dynamic_minimum[lo + nearest_dynamic_local]),
                }
            plans.append(BatchBasePlan(
                success, 'success' if success else 'base batch planning failed',
                0., batch, static_selection, dynamic_selection,
                item[1], item[2], item[3], failure_snapshot))
        phase5_time = time.perf_counter() - phase5_start
        elapsed = time.perf_counter() - total_start
        for plan in plans:
            plan.planning_time = elapsed / max(len(plans), 1)
        pool = SwarmCandidatePool(coefficients, owner, local, offsets,
                                  positions, velocities, accelerations)
        trajectory_bytes = positions.nbytes + velocities.nbytes + accelerations.nbytes
        timing = BatchPlanningTiming(
            generation_time, pool_time, validation_time, static_time, phase4_time,
            prediction_time, dynamic_time, phase5_time, elapsed, trajectory_bytes,
            trajectory_bytes + coefficients.nbytes + peak_distance_bytes,
            static_statistics['obstacles_considered'],
            static_statistics['exact_evaluations'],
            static_statistics['obstacles_skipped'],
            static_statistics['candidate_obstacle_pairs'])
        return plans, pool, timing
