import numpy as np
from math import sqrt


class Obstacle(object):
    """Static obstacle geometry in North-Up-East coordinates.

    Cuboids are axis-aligned with length along x/North, height along y/Up,
    and width along z/East. Cylinders are vertical along y/Up.
    """
    def __init__(self, pos, shape_dict, id):
        self.shape = shape = shape_dict['shape']
        if shape in ('cube', 'rotated_cube'):
            self.length, self.width, self.height = shape_dict['length'], shape_dict['width'], shape_dict['height']
            self.radius = sqrt(self.length ** 2 + self.width ** 2 + self.height ** 2) / 2
            self.angle = float(shape_dict.get('angle', 0.0))
        elif shape == 'sphere':
            self.feature = feature = shape_dict['feature']
            self.radius = shape_dict['feature']
        elif shape == 'cylinder':
            self.cylinder_radius = shape_dict.get('radius', shape_dict.get('feature'))
            self.height = shape_dict.get('height', 2.0 * self.cylinder_radius)
            self.feature = self.cylinder_radius
            self.radius = sqrt(self.cylinder_radius ** 2 + (self.height / 2.0) ** 2)
        else:
            raise NotImplementedError
        self.pos_global_frame = np.array(pos, dtype='float64')
        self.initial_pos_global_frame = self.pos_global_frame.copy()
        self.vel_global_frame = np.array([0.0, 0.0, 0.0])
        self.motion = shape_dict.get('motion')
        self.is_static = self.motion is None
        self.initial_angle = getattr(self, 'angle', 0.0)
        self.competition_id = shape_dict.get('competition_id', id)
        self.pos = pos
        self.id = id
        self.t = 0.0
        self.step_num = 0
        self.is_at_goal = True
        self.is_obstacle = True
        self.was_in_collision_already = False
        self.is_collision = False

        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]

    def update(self, time_seconds):
        """Move the obstacle to its deterministic position at ``time_seconds``.

        Motion is sinusoidal along a unit vector in the horizontal x-z plane.
        Static obstacles simply retain their initial position.
        """
        self.t = float(time_seconds)
        center, yaw = self.predict_pose(self.t)
        velocity = self.predict_velocity(self.t)
        self.pos_global_frame = np.asarray(center, dtype='float64')
        self.vel_global_frame = np.asarray(velocity, dtype='float64')
        if self.shape in ('cube', 'rotated_cube'):
            self.angle = float(yaw)
        self.x, self.y, self.z = self.pos_global_frame

    def _motion_direction(self):
        direction = np.asarray(self.motion['direction'], dtype='float64')
        norm = np.linalg.norm(direction)
        if norm <= 0.0:
            raise ValueError('obstacle motion requires a non-zero direction')
        return direction / norm

    def predict_pose(self, absolute_time):
        """Predict center and yaw at absolute mission time, scalar or array.

        The bundled sinusoidal motions are engineering demonstrations, not an
        official competition motion model.  Collision code depends only on
        this pose interface.
        """
        times = np.asarray(absolute_time, dtype='float64')
        if not self.motion:
            centers = np.broadcast_to(self.initial_pos_global_frame,
                                      times.shape + (3,)).copy()
            yaw = np.full(times.shape, self.initial_angle)
        elif self.motion.get('model') == 'constant_velocity':
            velocity = np.asarray(self.motion['velocity'], dtype='float64')
            centers = self.initial_pos_global_frame + times[..., None] * velocity
            yaw = (self.initial_angle +
                   float(self.motion.get('yaw_rate', 0.0)) * times)
        else:
            amplitude = float(self.motion['amplitude'])
            period = float(self.motion['period'])
            if period <= 0.0:
                raise ValueError('obstacle motion period must be positive')
            phase = float(self.motion.get('phase', 0.0))
            omega = 2.0 * np.pi / period
            argument = omega * times + phase
            centers = (self.initial_pos_global_frame +
                       amplitude * np.sin(argument)[..., None] *
                       self._motion_direction())
            yaw = (self.initial_angle +
                   float(self.motion.get('yaw_rate', 0.0)) * times)
        if times.ndim == 0:
            return centers.reshape(3), float(yaw)
        return centers, yaw

    def predict_velocity(self, absolute_time):
        """Analytic center velocity corresponding to ``predict_pose``."""
        times = np.asarray(absolute_time, dtype='float64')
        if not self.motion:
            velocity = np.zeros(times.shape + (3,), dtype='float64')
        elif self.motion.get('model') == 'constant_velocity':
            value = np.asarray(self.motion['velocity'], dtype='float64')
            velocity = np.broadcast_to(value, times.shape + (3,)).copy()
        else:
            amplitude = float(self.motion['amplitude'])
            period = float(self.motion['period'])
            if period <= 0.0:
                raise ValueError('obstacle motion period must be positive')
            phase = float(self.motion.get('phase', 0.0))
            omega = 2.0 * np.pi / period
            velocity = (amplitude * omega * np.cos(omega * times + phase)[..., None] *
                        self._motion_direction())
        return velocity.reshape(3) if times.ndim == 0 else velocity

    def predict_yaw_rate(self, absolute_time):
        times = np.asarray(absolute_time, dtype='float64')
        value = 0.0 if not self.motion else float(self.motion.get('yaw_rate', 0.0))
        result = np.full(times.shape, value, dtype='float64')
        return float(result) if times.ndim == 0 else result

    def translation_speed_bound(self):
        """Analytic global bound for the configured center-motion model."""
        if not self.motion:
            return 0.0
        if self.motion.get('model') == 'constant_velocity':
            return float(np.linalg.norm(self.motion['velocity']))
        period = float(self.motion['period'])
        if period <= 0.0:
            raise ValueError('obstacle motion period must be positive')
        return abs(float(self.motion['amplitude'])) * 2.0 * np.pi / period

    def distance_sq_to_point(self, point):
        """Squared distance from a point to the obstacle's solid volume."""
        return float(self.distance_sq_to_points(
            np.asarray(point, dtype='float64').reshape(1, 3))[0])

    def distance_sq_to_points(self, points):
        """Vectorized squared distance for points with shape (..., 3)."""
        points = np.asarray(points, dtype='float64')
        if points.shape[-1] != 3:
            raise ValueError('points must have final dimension 3')
        delta = points - self.pos_global_frame
        if self.shape == 'sphere':
            gap = np.maximum(0.0, np.linalg.norm(delta, axis=-1) - self.radius)
            return gap * gap
        if self.shape == 'cylinder':
            radial = np.sqrt(delta[..., 0] ** 2 + delta[..., 2] ** 2)
            radial_gap = np.maximum(0.0, radial - self.cylinder_radius)
            vertical_gap = np.maximum(0.0, np.abs(delta[..., 1]) - self.height / 2.0)
            return radial_gap ** 2 + vertical_gap ** 2
        horizontal = self._world_to_local_horizontal(delta[..., [0, 2]])
        local_delta = np.stack((horizontal[..., 0], delta[..., 1],
                                horizontal[..., 1]), axis=-1)
        half_extents = np.array([self.length / 2.0, self.height / 2.0, self.width / 2.0])
        gap = np.maximum(np.abs(local_delta) - half_extents, 0.0)
        return np.einsum('...i,...i->...', gap, gap)

    def _world_to_local_horizontal(self, horizontal):
        if self.shape not in ('cube', 'rotated_cube') or abs(self.angle) < 1e-15:
            return np.asarray(horizontal, dtype='float64')
        cosine, sine = np.cos(self.angle), np.sin(self.angle)
        return np.stack((cosine * horizontal[..., 0] + sine * horizontal[..., 1],
                         -sine * horizontal[..., 0] + cosine * horizontal[..., 1]),
                        axis=-1)

    def _local_to_world_horizontal(self, horizontal):
        if self.shape not in ('cube', 'rotated_cube') or abs(self.angle) < 1e-15:
            return np.asarray(horizontal, dtype='float64')
        cosine, sine = np.cos(self.angle), np.sin(self.angle)
        return np.array([cosine * horizontal[0] - sine * horizontal[1],
                         sine * horizontal[0] + cosine * horizontal[1]])

    def collides_with_sphere(self, center, radius):
        return self.distance_sq_to_point(center) <= radius ** 2

    def nearest_point(self, point):
        """Return the closest point on/in the actual obstacle geometry."""
        point = np.asarray(point, dtype='float64')
        delta = point - self.pos_global_frame
        if self.shape in ('cube', 'rotated_cube'):
            half = np.array([self.length / 2.0, self.height / 2.0, self.width / 2.0])
            horizontal = self._world_to_local_horizontal(delta[[0, 2]])
            local = np.array([horizontal[0], delta[1], horizontal[1]])
            nearest_local = np.clip(local, -half, half)
            nearest_horizontal = self._local_to_world_horizontal(nearest_local[[0, 2]])
            return self.pos_global_frame + np.array(
                [nearest_horizontal[0], nearest_local[1], nearest_horizontal[1]])
        if self.shape == 'sphere':
            norm = np.linalg.norm(delta)
            if norm <= self.radius or norm < 1e-12:
                return point.copy()
            return self.pos_global_frame + delta * (self.radius / norm)
        y = np.clip(point[1], self.pos_global_frame[1] - self.height / 2.0,
                    self.pos_global_frame[1] + self.height / 2.0)
        radial = delta[[0, 2]]
        radial_norm = np.linalg.norm(radial)
        if radial_norm > self.cylinder_radius:
            radial = radial * (self.cylinder_radius / radial_norm)
        return np.array([self.pos_global_frame[0] + radial[0], y,
                         self.pos_global_frame[2] + radial[1]])
