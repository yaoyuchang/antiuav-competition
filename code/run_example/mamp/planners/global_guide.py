"""Sparse 3-D Weighted A* Global Guide over static obstacles only."""

from dataclasses import dataclass
import heapq
import itertools
import math
import time

import numpy as np

from ..configs import subject3_config as config


@dataclass
class GlobalGuideResult:
    """Reusable guide path and planning diagnostics; contains no core progress state."""
    success: bool
    waypoints: np.ndarray
    raw_grid_path: np.ndarray
    raw_grid_indices: tuple
    raw_grid_path_length: float
    pruned_path_length: float
    expanded_nodes: int
    search_time: float
    pruning_time: float
    total_global_guide_time: float
    occupancy_build_time: float
    reason: str
    guide: object = None


# Phase-2 compatibility keeps the old result name; new code may use the clearer
# path-oriented name. Progress belongs to GlobalGuideTracker, not this object.
GlobalGuidePath = GlobalGuideResult


def path_length(points):
    points = np.asarray(points, dtype='float64')
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


class GlobalGuide(object):
    """Stateful monotone-progress query over pruned xyz waypoints."""

    def __init__(self, waypoints, lookahead_distance=None,
                 turn_aware_enabled=None, turn_accel_ref_ratio=None):
        self.waypoints = np.asarray(waypoints, dtype='float64')
        if self.waypoints.ndim != 2 or self.waypoints.shape[1] != 3:
            raise ValueError('waypoints must have shape (M, 3)')
        if len(self.waypoints) < 2:
            raise ValueError('at least two waypoints are required')
        self.lookahead_distance = (
            config.GLOBAL_LOOKAHEAD_DISTANCE if lookahead_distance is None
            else float(lookahead_distance))
        self.segment_lengths = np.linalg.norm(np.diff(self.waypoints, axis=0), axis=1)
        self.current_segment_index = 0
        self.last_target = None
        self.turn_aware_enabled = (config.ENABLE_TURN_AWARE_PREVIEW
            if turn_aware_enabled is None else bool(turn_aware_enabled))
        self.turn_accel_ref_ratio = (config.GUIDE_TURN_ACCEL_REF_RATIO
            if turn_accel_ref_ratio is None else float(turn_accel_ref_ratio))
        if self.turn_accel_ref_ratio <= 0.0:
            raise ValueError('turn_accel_ref_ratio must be positive')
        self.last_turn_diagnostics = None

    def reset_progress(self):
        self.current_segment_index = 0
        self.last_turn_diagnostics = None

    def _projection(self, position, segment_index):
        start = self.waypoints[segment_index]
        delta = self.waypoints[segment_index + 1] - start
        length_sq = float(np.dot(delta, delta))
        if length_sq <= 1e-16:
            return start.copy(), 1.0
        fraction = float(np.clip(np.dot(position - start, delta) / length_sq, 0.0, 1.0))
        return start + fraction * delta, fraction

    def get_global_direction(self, position, lookahead_distance=None):
        """Return an xyz unit vector while never decreasing progress index."""
        position = np.asarray(position, dtype='float64')
        if position.shape != (3,):
            raise ValueError('position must have shape (3,)')
        candidates = []
        for index in range(self.current_segment_index, len(self.waypoints) - 1):
            projected, fraction = self._projection(position, index)
            candidates.append((float(np.linalg.norm(position - projected)),
                               -index, index, projected, fraction))
        _, _, best_index, projected, fraction = min(candidates, key=lambda item: item[:2])
        self.current_segment_index = max(self.current_segment_index, best_index)

        remaining = (self.lookahead_distance if lookahead_distance is None
                     else float(lookahead_distance))
        current_length = self.segment_lengths[self.current_segment_index]
        available = (1.0 - fraction) * current_length
        if remaining <= available and current_length > 1e-16:
            direction = (self.waypoints[self.current_segment_index + 1] -
                         self.waypoints[self.current_segment_index]) / current_length
            target = projected + remaining * direction
        else:
            remaining -= available
            target = self.waypoints[self.current_segment_index + 1].copy()
            for index in range(self.current_segment_index + 1, len(self.segment_lengths)):
                length = self.segment_lengths[index]
                if remaining <= length and length > 1e-16:
                    target = (self.waypoints[index] + remaining / length *
                              (self.waypoints[index + 1] - self.waypoints[index]))
                    break
                remaining -= length
                target = self.waypoints[index + 1].copy()
        delta = target - position
        self.last_target = target.copy()
        norm = float(np.linalg.norm(delta))
        return delta / norm if norm > 1e-12 else np.zeros(3, dtype='float64')

    def get_turn_aware_direction(self, position, velocity, lookahead_distance=None):
        """Blend xz headings before a corner while preserving old tracker semantics."""
        position = np.asarray(position, dtype='float64')
        velocity = np.asarray(velocity, dtype='float64')
        if velocity.shape != (3,):
            raise ValueError('velocity must have shape (3,)')
        original = self.get_global_direction(position, lookahead_distance)
        preview_start = time.perf_counter()
        index = self.current_segment_index
        diagnostics = {
            'current_segment_index': int(index), 'distance_to_corner': float('inf'),
            'corner_angle_deg': 0.0,
            'horizontal_speed': float(np.linalg.norm(velocity[[0, 2]])),
            'R_ref': 0.0, 'D_preview': 0.0, 'beta': 0.0,
            'original_guide_direction': original.copy(),
            'turn_aware_guide_direction': original.copy(),
            'turn_aware_enabled': bool(self.turn_aware_enabled),
            'turn_aware_computation_time': 0.0}
        if not self.turn_aware_enabled or index + 2 >= len(self.waypoints):
            diagnostics['turn_aware_computation_time'] = time.perf_counter() - preview_start
            self.last_turn_diagnostics = diagnostics
            return original
        incoming = self.waypoints[index + 1, [0, 2]] - self.waypoints[index, [0, 2]]
        outgoing = self.waypoints[index + 2, [0, 2]] - self.waypoints[index + 1, [0, 2]]
        incoming_length = float(np.linalg.norm(incoming))
        outgoing_length = float(np.linalg.norm(outgoing))
        if incoming_length <= 1.0e-12 or outgoing_length <= 1.0e-12:
            diagnostics['turn_aware_computation_time'] = time.perf_counter() - preview_start
            self.last_turn_diagnostics = diagnostics
            return original
        t_in, t_out = incoming / incoming_length, outgoing / outgoing_length
        angle = math.acos(float(np.clip(np.dot(t_in, t_out), -1.0, 1.0)))
        _, fraction = self._projection(position, index)
        distance_to_corner = (1.0 - fraction) * self.segment_lengths[index]
        diagnostics.update({'corner_angle_deg': math.degrees(angle),
                            'distance_to_corner': float(distance_to_corner)})
        if angle <= config.TURN_ANGLE_EPS:
            diagnostics['turn_aware_computation_time'] = time.perf_counter() - preview_start
            self.last_turn_diagnostics = diagnostics
            return original
        acceleration_ref = max(self.turn_accel_ref_ratio * config.A_XZ_MAX, 1.0e-12)
        radius_ref = diagnostics['horizontal_speed'] ** 2 / acceleration_ref
        preview = float(np.clip(radius_ref * math.tan(0.5 * angle),
                                config.TURN_PREVIEW_MIN, config.TURN_PREVIEW_MAX))
        s = float(np.clip(1.0 - distance_to_corner / preview, 0.0, 1.0))
        beta = s * s * (3.0 - 2.0 * s)
        diagnostics.update({'R_ref': radius_ref, 'D_preview': preview, 'beta': beta})
        if beta > 0.0:
            blended = (1.0 - beta) * t_in + beta * t_out
            blended_norm = float(np.linalg.norm(blended))
            if blended_norm <= 1.0e-12:
                blended, blended_norm = (t_in if beta < 0.5 else t_out), 1.0
            blended /= blended_norm
            horizontal = math.sqrt(max(0.0, 1.0 - original[1] ** 2))
            aware = np.array([horizontal * blended[0], original[1],
                              horizontal * blended[1]], dtype='float64')
            aware_norm = float(np.linalg.norm(aware))
            if aware_norm > 1.0e-12:
                diagnostics['turn_aware_guide_direction'] = aware / aware_norm
        diagnostics['turn_aware_computation_time'] = time.perf_counter() - preview_start
        self.last_turn_diagnostics = diagnostics
        return diagnostics['turn_aware_guide_direction'].copy()

    def get_remaining_path_length(self, position):
        """Return projected remaining guide length without changing progress."""
        position = np.asarray(position, dtype='float64')
        if position.shape != (3,):
            raise ValueError('position must have shape (3,)')
        candidates = []
        for index in range(self.current_segment_index, len(self.waypoints) - 1):
            projected, fraction = self._projection(position, index)
            candidates.append((float(np.linalg.norm(position - projected)),
                               -index, index, fraction))
        _, _, index, fraction = min(candidates, key=lambda item: item[:2])
        remaining = (1.0 - fraction) * self.segment_lengths[index]
        if index + 1 < len(self.segment_lengths):
            remaining += float(np.sum(self.segment_lengths[index + 1:]))
        return float(remaining)


class GlobalGuideTracker(GlobalGuide):
    """Independent progress state over shared immutable guide waypoints."""

    @classmethod
    def from_result(cls, result, lookahead_distance=None,
                    turn_aware_enabled=None, turn_accel_ref_ratio=None):
        if not result.success:
            raise ValueError('cannot track an unsuccessful guide')
        return cls(result.waypoints, lookahead_distance, turn_aware_enabled,
                   turn_accel_ref_ratio)


class GlobalGuidePlanner(object):
    """Cached static occupancy plus single-UAV Weighted A* queries."""

    def __init__(self, obstacles, boundary_points, dx=None, dy=None, dz=None,
                 boundary_margin=None, w_astar=None, edge_sample_spacing=None,
                 vertical_reserve=None):
        self.obstacles = list(obstacles)
        self.static_obstacles = [obstacle for obstacle in self.obstacles
                                 if obstacle.is_static]
        self.dynamic_obstacles = [obstacle for obstacle in self.obstacles
                                  if not obstacle.is_static]
        self.spacing = np.array([
            config.GLOBAL_DX if dx is None else float(dx),
            config.GLOBAL_DY if dy is None else float(dy),
            config.GLOBAL_DZ if dz is None else float(dz)], dtype='float64')
        if np.any(self.spacing <= 0.0):
            raise ValueError('grid spacing must be positive')
        self.boundary_margin = (config.GLOBAL_BOUNDARY_MARGIN if boundary_margin is None
                                else float(boundary_margin))
        self.w_astar = config.GLOBAL_W_ASTAR if w_astar is None else float(w_astar)
        self.edge_sample_spacing = (
            config.GLOBAL_EDGE_SAMPLE_SPACING if edge_sample_spacing is None
            else float(edge_sample_spacing))
        self.vertical_reserve = float(
            config.GLOBAL_GUIDE_VERTICAL_RESERVE if vertical_reserve is None
            else vertical_reserve)
        self.guide_y_max = config.Y_MAX - self.vertical_reserve
        if self.vertical_reserve < 0.0 or self.guide_y_max < config.Y_MIN_ASSUMED:
            raise ValueError('vertical_reserve leaves an invalid guide workspace')
        if self.edge_sample_spacing > 0.5 * float(np.min(self.spacing)):
            raise ValueError('edge sample spacing exceeds half the smallest grid spacing')
        self.origin, maximum = self._build_bounds(boundary_points)
        counts = np.rint((maximum - self.origin) / self.spacing).astype(int) + 1
        self.x_coords = self.origin[0] + np.arange(counts[0]) * self.spacing[0]
        self.y_coords = self.origin[1] + np.arange(counts[1]) * self.spacing[1]
        self.z_coords = self.origin[2] + np.arange(counts[2]) * self.spacing[2]
        build_start = time.perf_counter()
        self.occupied = self._build_static_occupancy()
        self.occupancy_build_time = time.perf_counter() - build_start
        edge_start = time.perf_counter()
        self.edge_directions = self._canonical_edge_directions()
        self.edge_valid = self._build_edge_validity()
        self.edge_validity_build_time = time.perf_counter() - edge_start
        self.static_preprocess_time = (self.occupancy_build_time +
                                       self.edge_validity_build_time)
        self.edge_validity_bytes = int(self.edge_valid.nbytes)
        self.edge_validity_edge_count = self._count_geometric_edges()
        self.edge_query_count = 0
        self._search_neighbors = self._build_search_neighbors()
        self.last_result = None

    def _obstacle_horizontal_extent(self, obstacle):
        if obstacle.shape == 'cylinder':
            return obstacle.cylinder_radius, obstacle.cylinder_radius
        if obstacle.shape == 'sphere':
            return obstacle.radius, obstacle.radius
        angle = getattr(obstacle, 'angle', 0.0)
        cosine, sine = abs(math.cos(angle)), abs(math.sin(angle))
        return (cosine * obstacle.length / 2.0 + sine * obstacle.width / 2.0,
                sine * obstacle.length / 2.0 + cosine * obstacle.width / 2.0)

    def _build_bounds(self, boundary_points):
        points = np.asarray(boundary_points, dtype='float64')
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError('boundary_points must have shape (N, 3)')
        x_values = list(points[:, 0])
        z_values = list(points[:, 2])
        for obstacle in self.static_obstacles:
            extent_x, extent_z = self._obstacle_horizontal_extent(obstacle)
            x_values.extend((obstacle.x - extent_x, obstacle.x + extent_x))
            z_values.extend((obstacle.z - extent_z, obstacle.z + extent_z))
        lower = np.array([
            math.floor((min(x_values) - self.boundary_margin) / self.spacing[0]) * self.spacing[0],
            config.Y_MIN_ASSUMED,
            math.floor((min(z_values) - self.boundary_margin) / self.spacing[2]) * self.spacing[2]])
        upper = np.array([
            math.ceil((max(x_values) + self.boundary_margin) / self.spacing[0]) * self.spacing[0],
            self.guide_y_max,
            math.ceil((max(z_values) + self.boundary_margin) / self.spacing[2]) * self.spacing[2]])
        return lower, upper

    def _build_static_occupancy(self):
        shape = (len(self.x_coords), len(self.y_coords), len(self.z_coords))
        occupied = np.zeros(shape, dtype=bool)
        margin = config.OBS_SAFE_DISTANCE
        x_grid = self.x_coords[:, None]
        z_grid = self.z_coords[None, :]
        for obstacle in self.static_obstacles:
            vertical = np.abs(self.y_coords - obstacle.y) <= obstacle.height / 2.0 + margin
            if obstacle.shape == 'cylinder':
                horizontal = ((x_grid - obstacle.x) ** 2 +
                              (z_grid - obstacle.z) ** 2 <=
                              (obstacle.cylinder_radius + margin) ** 2)
            elif obstacle.shape == 'sphere':
                dx = self.x_coords[:, None, None] - obstacle.x
                dy = self.y_coords[None, :, None] - obstacle.y
                dz = self.z_coords[None, None, :] - obstacle.z
                occupied |= dx * dx + dy * dy + dz * dz <= (obstacle.radius + margin) ** 2
                continue
            else:
                dx = x_grid - obstacle.x
                dz = z_grid - obstacle.z
                angle = getattr(obstacle, 'angle', 0.0)
                local_x = math.cos(angle) * dx + math.sin(angle) * dz
                local_z = -math.sin(angle) * dx + math.cos(angle) * dz
                horizontal = ((np.abs(local_x) <= obstacle.length / 2.0 + margin) &
                              (np.abs(local_z) <= obstacle.width / 2.0 + margin))
            occupied |= horizontal[:, None, :] & vertical[None, :, None]
        return occupied

    def world_to_grid(self, point):
        point = np.asarray(point, dtype='float64')
        index = np.rint((point - self.origin) / self.spacing).astype(int)
        maximum = np.array(self.occupied.shape) - 1
        return tuple(np.clip(index, 0, maximum).tolist())

    def grid_to_world(self, index):
        return self.origin + np.asarray(index, dtype='float64') * self.spacing

    def points_are_static_safe(self, points):
        points = np.asarray(points, dtype='float64')
        if points.shape[-1] != 3:
            raise ValueError('points must have final dimension 3')
        altitude_safe = ((points[..., 1] >= config.Y_MIN_ASSUMED) &
                         (points[..., 1] <= config.Y_MAX))
        safe = altitude_safe.copy()
        margin_sq = config.OBS_SAFE_DISTANCE ** 2
        for obstacle in self.static_obstacles:
            safe &= obstacle.distance_sq_to_points(points) >= margin_sq - 1e-10
        return safe

    def segment_is_static_safe(self, start, end):
        start = np.asarray(start, dtype='float64')
        end = np.asarray(end, dtype='float64')
        length = float(np.linalg.norm(end - start))
        samples = max(2, int(math.ceil(length / self.edge_sample_spacing)) + 1)
        fractions = np.linspace(0.0, 1.0, samples)
        points = start[None, :] + fractions[:, None] * (end - start)[None, :]
        return bool(np.all(self.points_are_static_safe(points)))

    def _grid_edge_is_safe(self, current, neighbor):
        offset = tuple(np.asarray(neighbor) - np.asarray(current))
        lookup = self._edge_direction_lookup.get(offset)
        if lookup is None:
            raise ValueError('nodes are not 26-connected neighbors')
        direction_index, reverse = lookup
        base = neighbor if reverse else current
        self.edge_query_count += 1
        return bool(self.edge_valid[(direction_index,) + tuple(base)])

    def _count_geometric_edges(self):
        total = 0
        for direction in self.edge_directions:
            ranges = self._valid_start_ranges(direction)
            total += int(np.prod([hi - lo + 1 for lo, hi in ranges]))
        return total

    def _build_search_neighbors(self):
        """Precompute scalar search data after profiling exposed tiny-array overhead."""
        result = []
        for offset in itertools.product((-1, 0, 1), repeat=3):
            if offset == (0, 0, 0):
                continue
            dx, dy, dz = offset
            cost = math.sqrt((dx * self.spacing[0]) ** 2 +
                             (dy * self.spacing[1]) ** 2 +
                             (dz * self.spacing[2]) ** 2)
            direction_index, reverse = self._edge_direction_lookup[offset]
            result.append((dx, dy, dz, cost, direction_index, reverse))
        return tuple(result)

    @staticmethod
    def _canonical_edge_directions():
        directions = []
        for offset in itertools.product((-1, 0, 1), repeat=3):
            if offset == (0, 0, 0):
                continue
            first_nonzero = next(value for value in offset if value != 0)
            if first_nonzero > 0:
                directions.append(offset)
        return tuple(directions)

    def _valid_start_ranges(self, direction):
        ranges = []
        for size, delta in zip(self.occupied.shape, direction):
            if delta > 0:
                ranges.append((0, size - delta - 1))
            elif delta < 0:
                ranges.append((-delta, size - 1))
            else:
                ranges.append((0, size - 1))
        return ranges

    def _obstacle_inflated_bounds(self, obstacle):
        margin = config.OBS_SAFE_DISTANCE
        extent_x, extent_z = self._obstacle_horizontal_extent(obstacle)
        if obstacle.shape == 'sphere':
            extent_y = obstacle.radius
        else:
            extent_y = obstacle.height / 2.0
        lower = np.array([obstacle.x - extent_x - margin,
                          obstacle.y - extent_y - margin,
                          obstacle.z - extent_z - margin])
        upper = np.array([obstacle.x + extent_x + margin,
                          obstacle.y + extent_y + margin,
                          obstacle.z + extent_z + margin])
        return lower, upper

    def _candidate_index_ranges(self, obstacle, direction, valid_ranges):
        lower, upper = self._obstacle_inflated_bounds(obstacle)
        world_delta = np.asarray(direction, dtype='float64') * self.spacing
        # A start is relevant when the AABB of its edge can overlap the
        # obstacle's inflated AABB.
        start_lower = lower - np.maximum(world_delta, 0.0)
        start_upper = upper - np.minimum(world_delta, 0.0)
        index_lower = np.ceil((start_lower - self.origin) / self.spacing).astype(int)
        index_upper = np.floor((start_upper - self.origin) / self.spacing).astype(int)
        clipped = []
        for axis in range(3):
            lo = max(int(index_lower[axis]), valid_ranges[axis][0])
            hi = min(int(index_upper[axis]), valid_ranges[axis][1])
            if lo > hi:
                return None
            clipped.append((lo, hi))
        return clipped

    def _build_edge_validity(self):
        shape = (len(self.edge_directions),) + self.occupied.shape
        edge_valid = np.zeros(shape, dtype=bool)
        self._edge_direction_lookup = {}
        margin_sq = config.OBS_SAFE_DISTANCE ** 2
        for direction_index, direction in enumerate(self.edge_directions):
            self._edge_direction_lookup[direction] = (direction_index, False)
            reverse = tuple(-value for value in direction)
            self._edge_direction_lookup[reverse] = (direction_index, True)
            valid_ranges = self._valid_start_ranges(direction)
            valid_slice = tuple(slice(lo, hi + 1) for lo, hi in valid_ranges)
            edge_valid[(direction_index,) + valid_slice] = True
            world_delta = np.asarray(direction, dtype='float64') * self.spacing
            edge_length = float(np.linalg.norm(world_delta))
            sample_count = max(2, int(math.ceil(
                edge_length / self.edge_sample_spacing)) + 1)
            fractions = np.linspace(0.0, 1.0, sample_count)
            for obstacle in self.static_obstacles:
                ranges = self._candidate_index_ranges(
                    obstacle, direction, valid_ranges)
                if ranges is None:
                    continue
                axes = [np.arange(lo, hi + 1, dtype=int) for lo, hi in ranges]
                mesh = np.meshgrid(*axes, indexing='ij')
                indices = np.column_stack([values.ravel() for values in mesh])
                starts = self.origin + indices * self.spacing
                samples = (starts[:, None, :] +
                           fractions[None, :, None] * world_delta[None, None, :])
                safe = np.all(
                    obstacle.distance_sq_to_points(samples) >= margin_sq - 1e-10,
                    axis=1)
                target = ((direction_index, indices[:, 0], indices[:, 1],
                           indices[:, 2]))
                edge_valid[target] &= safe
        return edge_valid

    def _endpoint_grid_node(self, point):
        nearest = np.asarray(self.world_to_grid(point), dtype=int)
        maximum = np.array(self.occupied.shape) - 1
        candidates = []
        radius_limit = config.GLOBAL_ENDPOINT_SEARCH_CELLS
        for offsets in itertools.product(range(-radius_limit, radius_limit + 1), repeat=3):
            candidate = nearest + np.asarray(offsets)
            if np.any(candidate < 0) or np.any(candidate > maximum):
                continue
            index = tuple(candidate.tolist())
            if self.occupied[index]:
                continue
            world = self.grid_to_world(index)
            candidates.append((float(np.linalg.norm(world - point)), index, world))
        candidates.sort(key=lambda item: item[0])
        for _, index, world in candidates:
            if self.segment_is_static_safe(point, world):
                return index
        return None

    def _neighbor_steps(self):
        result = []
        for offset in itertools.product((-1, 0, 1), repeat=3):
            if offset == (0, 0, 0):
                continue
            world_delta = np.asarray(offset, dtype='float64') * self.spacing
            result.append((offset, float(np.linalg.norm(world_delta))))
        return result

    def _weighted_astar(self, start_index, goal_index):
        search_start = time.perf_counter()
        self.edge_query_count = 0
        shape = self.occupied.shape
        g_score = np.full(shape, np.inf, dtype='float64')
        closed = np.zeros(shape, dtype=bool)
        g_score[start_index] = 0.0
        queue = []
        counter = itertools.count()

        goal_x, goal_y, goal_z = goal_index
        sx, sy, sz = (float(value) for value in self.spacing)

        def heuristic(index):
            dx = (index[0] - goal_x) * sx
            dy = (index[1] - goal_y) * sy
            dz = (index[2] - goal_z) * sz
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        heapq.heappush(queue, (self.w_astar * heuristic(start_index),
                               next(counter), 0.0, start_index))
        came_from = {}
        expanded = 0
        max_x, max_y, max_z = shape[0] - 1, shape[1] - 1, shape[2] - 1
        found = False
        while queue:
            _, _, queued_g, current = heapq.heappop(queue)
            if closed[current] or queued_g > g_score[current] + 1e-12:
                continue
            if current == goal_index:
                found = True
                break
            closed[current] = True
            expanded += 1
            current_x, current_y, current_z = current
            for dx, dy, dz, edge_cost, direction_index, reverse in self._search_neighbors:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                neighbor_z = current_z + dz
                if (neighbor_x < 0 or neighbor_x > max_x or
                        neighbor_y < 0 or neighbor_y > max_y or
                        neighbor_z < 0 or neighbor_z > max_z):
                    continue
                neighbor = (neighbor_x, neighbor_y, neighbor_z)
                if closed[neighbor] or self.occupied[neighbor]:
                    continue
                tentative = queued_g + edge_cost
                if tentative >= g_score[neighbor]:
                    continue
                base = neighbor if reverse else current
                self.edge_query_count += 1
                if not self.edge_valid[(direction_index,) + base]:
                    continue
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                priority = tentative + self.w_astar * heuristic(neighbor)
                heapq.heappush(queue, (priority, next(counter), tentative, neighbor))

        elapsed = time.perf_counter() - search_start
        if not found:
            return (), expanded, elapsed
        path = [goal_index]
        while path[-1] != start_index:
            path.append(came_from[path[-1]])
        path.reverse()
        return tuple(path), expanded, elapsed

    def _prune(self, points):
        prune_start = time.perf_counter()
        if len(points) <= 2:
            return points.copy(), time.perf_counter() - prune_start
        retained = [points[0]]
        current = 0
        while current < len(points) - 1:
            next_index = current + 1
            for candidate in range(len(points) - 1, current, -1):
                workspace_safe = (max(points[current, 1], points[candidate, 1]) <=
                                  self.guide_y_max + 1.0e-10)
                if (workspace_safe and
                        self.segment_is_static_safe(points[current], points[candidate])):
                    next_index = candidate
                    break
            retained.append(points[next_index])
            current = next_index
        return np.asarray(retained), time.perf_counter() - prune_start

    @staticmethod
    def _failure(reason, occupancy_build_time, total_time, search_time=0.0,
                 expanded=0):
        empty = np.empty((0, 3), dtype='float64')
        return GlobalGuideResult(False, empty, empty, (), 0.0, 0.0, expanded,
                                 search_time, 0.0, total_time,
                                 occupancy_build_time, reason, None)

    def plan(self, start, goal):
        total_start = time.perf_counter()
        start = np.asarray(start, dtype='float64')
        goal = np.asarray(goal, dtype='float64')
        if start.shape != (3,) or goal.shape != (3,):
            raise ValueError('start and goal must have shape (3,)')
        if not bool(self.points_are_static_safe(start)):
            return self._failure('start is outside altitude bounds or static safety margin',
                                 self.occupancy_build_time,
                                 time.perf_counter() - total_start)
        if not bool(self.points_are_static_safe(goal)):
            return self._failure('goal is outside altitude bounds or static safety margin',
                                 self.occupancy_build_time,
                                 time.perf_counter() - total_start)
        start_index = self._endpoint_grid_node(start)
        if start_index is None:
            return self._failure('no free grid node connects safely to exact start',
                                 self.occupancy_build_time,
                                 time.perf_counter() - total_start)
        goal_index = self._endpoint_grid_node(goal)
        if goal_index is None:
            return self._failure('no free grid node connects safely to exact goal',
                                 self.occupancy_build_time,
                                 time.perf_counter() - total_start)

        grid_indices, expanded, search_time = self._weighted_astar(
            start_index, goal_index)
        if not grid_indices:
            return self._failure('Weighted A* found no static path',
                                 self.occupancy_build_time,
                                 time.perf_counter() - total_start,
                                 search_time, expanded)
        raw_grid_path = np.asarray([self.grid_to_world(index)
                                    for index in grid_indices])
        raw_points = np.vstack((start, raw_grid_path, goal))
        keep = np.r_[True, np.linalg.norm(np.diff(raw_points, axis=0), axis=1) > 1e-10]
        raw_points = raw_points[keep]
        endpoint_outside_workspace = (
            start[1] > self.guide_y_max + 1.0e-10 or
            goal[1] > self.guide_y_max + 1.0e-10)
        if endpoint_outside_workspace:
            # Preserve exact endpoint connectors.  Pruning only the workspace
            # nodes prevents a long LOS shortcut from escaping above the
            # operational guide ceiling.
            pruned_grid, pruning_time = self._prune(raw_grid_path)
            waypoints = np.vstack((start, pruned_grid, goal))
            keep_waypoint = np.r_[
                True, np.linalg.norm(np.diff(waypoints, axis=0), axis=1) > 1e-10]
            waypoints = waypoints[keep_waypoint]
        else:
            waypoints, pruning_time = self._prune(raw_points)
        result = GlobalGuideResult(
            True, waypoints, raw_grid_path, grid_indices,
            path_length(raw_points), path_length(waypoints), expanded,
            search_time, pruning_time, time.perf_counter() - total_start,
            self.occupancy_build_time, 'success',
            GlobalGuide(waypoints))
        self.last_result = result
        return result

    def get_global_direction(self, position, lookahead_distance=None):
        if self.last_result is None or not self.last_result.success:
            raise RuntimeError('plan() must succeed before querying direction')
        return self.last_result.guide.get_global_direction(position, lookahead_distance)

    @staticmethod
    def create_trackers(result, count, lookahead_distance=None):
        """Create independent trackers sharing one result's waypoint array."""
        return [GlobalGuideTracker.from_result(result, lookahead_distance)
                for _ in range(int(count))]
