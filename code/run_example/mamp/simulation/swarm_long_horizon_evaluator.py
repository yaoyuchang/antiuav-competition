"""Long-horizon safety and planning statistics for synchronized swarms."""

from dataclasses import dataclass

import numpy as np

from ..collision.dynamic_primitive_collision import distance_to_predicted_obstacle
from ..collision.swarm_trajectory_collision import interval_minimum
from ..configs import subject3_config as config


@dataclass
class ExecutedSegmentAudit:
    minimum_uav_distance: float
    closest_uav_pair: object
    uav_violation_count: int
    minimum_static_clearance: float
    closest_static_obstacle_id: int
    static_violation_count: int
    minimum_dynamic_clearance: float
    closest_dynamic_obstacle_id: int
    dynamic_violation_count: int


class SwarmLongHorizonEvaluator(object):
    """Audit executed prefixes independently of the planning fast path."""

    def __init__(self, obstacles):
        self.static_obstacles = tuple(x for x in obstacles if x.is_static)
        self.dynamic_obstacles = tuple(x for x in obstacles if not x.is_static)

    def audit_segment(self, positions, absolute_times):
        positions = np.asarray(positions, dtype='float64')
        times = np.asarray(absolute_times, dtype='float64')
        count = len(positions)
        pair_i, pair_j = np.triu_indices(count, k=1)
        if len(pair_i):
            pair_distance, _ = interval_minimum(
                positions[pair_i] - positions[pair_j], times)
            best_pair = int(np.argmin(pair_distance))
            minimum_pair = float(pair_distance[best_pair])
            closest_pair = (int(pair_i[best_pair]), int(pair_j[best_pair]))
            pair_violations = int(np.count_nonzero(
                pair_distance < config.UAV_SAFE_DISTANCE - 1.e-8))
        else:
            minimum_pair, closest_pair, pair_violations = np.inf, None, 0

        static_minimum, static_id, static_violations = np.inf, -1, 0
        flattened = positions.reshape((-1, 3))
        for obstacle in self.static_obstacles:
            value = float(np.min(np.sqrt(obstacle.distance_sq_to_points(flattened))))
            if value < static_minimum:
                static_minimum = value
                static_id = int(obstacle.competition_id)
            static_violations += int(value < config.OBS_SAFE_DISTANCE - 1.e-8)

        dynamic_minimum, dynamic_id, dynamic_violations = np.inf, -1, 0
        for obstacle in self.dynamic_obstacles:
            centers, yaw = obstacle.predict_pose(times)
            distance = distance_to_predicted_obstacle(positions, obstacle, centers, yaw)
            value = float(np.min(distance))
            if value < dynamic_minimum:
                dynamic_minimum = value
                dynamic_id = int(obstacle.competition_id)
            dynamic_violations += int(value < config.OBS_SAFE_DISTANCE - 1.e-8)
        return ExecutedSegmentAudit(
            minimum_pair, closest_pair, pair_violations,
            static_minimum, static_id, static_violations,
            dynamic_minimum, dynamic_id, dynamic_violations)

