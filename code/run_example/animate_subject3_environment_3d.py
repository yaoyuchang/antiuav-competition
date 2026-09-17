"""Interactive 3-D animation of the subject-3 obstacle environment."""

import argparse
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from mamp.envs.subject3_environment import Subject3Environment


BOX_FACES = (
    (0, 1, 2, 3), (4, 5, 6, 7),
    (0, 1, 5, 4), (2, 3, 7, 6),
    (1, 2, 6, 5), (0, 3, 7, 4),
)


def display_points(points):
    """Convert world NUE [x, y, z] to Matplotlib [x, z, y]."""
    points = np.asarray(points, dtype='float64')
    return points[:, [0, 2, 1]]


def box_faces(obstacle):
    half_length = obstacle.length / 2.0
    half_width = obstacle.width / 2.0
    half_height = obstacle.height / 2.0
    angle = getattr(obstacle, 'angle', 0.0)
    cosine, sine = math.cos(angle), math.sin(angle)
    vertices = []
    for local_x, local_z, local_y in (
            (-half_length, -half_width, -half_height),
            (half_length, -half_width, -half_height),
            (half_length, half_width, -half_height),
            (-half_length, half_width, -half_height),
            (-half_length, -half_width, half_height),
            (half_length, -half_width, half_height),
            (half_length, half_width, half_height),
            (-half_length, half_width, half_height)):
        world_x = obstacle.x + cosine * local_x - sine * local_z
        world_z = obstacle.z + sine * local_x + cosine * local_z
        vertices.append([world_x, obstacle.y + local_y, world_z])
    displayed = display_points(vertices)
    return [[displayed[index] for index in face] for face in BOX_FACES]


def draw_cylinder(axis, obstacle, color):
    theta = np.linspace(0.0, 2.0 * np.pi, 32)
    height = np.array([0.0, obstacle.height])
    theta_grid, height_grid = np.meshgrid(theta, height)
    north = obstacle.x + obstacle.cylinder_radius * np.cos(theta_grid)
    east = obstacle.z + obstacle.cylinder_radius * np.sin(theta_grid)
    axis.plot_surface(north, east, height_grid, color=color, alpha=0.58,
                      linewidth=0, shade=True)
    for altitude in (0.0, obstacle.height):
        axis.plot_trisurf(
            obstacle.x + obstacle.cylinder_radius * np.cos(theta),
            obstacle.z + obstacle.cylinder_radius * np.sin(theta),
            np.full_like(theta, altitude), color=color, alpha=0.58,
            linewidth=0)


def run_animation(speed=1.0, interval_ms=50, vertical_exaggeration=4.0):
    environment = Subject3Environment()
    figure = plt.figure(figsize=(15, 7))
    axis = figure.add_subplot(111, projection='3d')
    moving_artists = []
    labelled = set()

    for obstacle in environment.obstacles:
        dynamic = obstacle.motion is not None
        color = '#68c44f' if dynamic else '#3195c5'
        if obstacle.shape == 'cylinder':
            draw_cylinder(axis, obstacle, color)
        else:
            collection = Poly3DCollection(
                box_faces(obstacle), facecolors=color, edgecolors='#263238',
                alpha=0.62, linewidths=0.55)
            axis.add_collection3d(collection)
            if dynamic:
                moving_artists.append((obstacle, collection))
                direction = np.asarray(obstacle.motion['direction'])[[0, 2]]
                direction = direction / np.linalg.norm(direction)
                amplitude = obstacle.motion['amplitude']
                centre = obstacle.initial_pos_global_frame[[0, 2]]
                endpoints = np.vstack((centre - amplitude * direction,
                                        centre + amplitude * direction))
                axis.plot(endpoints[:, 0], endpoints[:, 1],
                          np.full(2, obstacle.height + 3.0), '--',
                          color='#287a1b', linewidth=1.0)

        if obstacle.competition_id not in labelled:
            axis.text(obstacle.x, obstacle.z, obstacle.height + 3.0,
                      str(obstacle.competition_id), ha='center', fontsize=7)
            labelled.add(obstacle.competition_id)

    starts_display = display_points(environment.starts)
    initial_display = display_points(environment.initial_goals)
    changed_display = display_points(environment.changed_goals)
    axis.scatter(starts_display[:, 0], starts_display[:, 1], starts_display[:, 2],
                 marker='o', s=24, color='lime', edgecolor='black',
                 linewidth=0.4, label='24 UAV starts')
    axis.scatter(initial_display[:, 0], initial_display[:, 1], initial_display[:, 2],
                 marker='o', s=25, color='#2457e6', label='initial goals')
    axis.scatter(changed_display[:, 0], changed_display[:, 1], changed_display[:, 2],
                 marker='x', s=34, color='#ff5a36', label='changed goals')
    axis.set_xlim(-100.0, 5100.0)
    axis.set_ylim(-650.0, 650.0)
    axis.set_zlim(0.0, 110.0)
    axis.set_xlabel('North x (m)')
    axis.set_ylabel('East z (m)')
    axis.set_zlabel('Up y (m)')
    axis.set_title('Subject 3 dynamic obstacle environment - 3D')
    # Strict physical proportions (5000:1300:110) make obstacle height almost
    # invisible.  Compress the course and exaggerate altitude for display only;
    # geometry queries and all stored coordinates remain in real metres.
    physical_ratio = np.array([5000.0, 1300.0,
                               110.0 * float(vertical_exaggeration)])
    desired_aspect = physical_ratio / physical_ratio[1]
    if hasattr(axis, 'set_box_aspect'):
        axis.set_box_aspect(desired_aspect)
    else:
        # Matplotlib bundled with some Python 3.8 installations predates
        # Axes3D.set_box_aspect.  Apply a projection-only fallback instead.
        original_projection = axis.get_proj
        softened_aspect = np.sqrt(desired_aspect / np.max(desired_aspect))
        projection_scale = np.diag([
            softened_aspect[0], softened_aspect[1], softened_aspect[2], 1.0])
        axis.get_proj = lambda: np.dot(original_projection(), projection_scale)
    axis.view_init(elev=23.0, azim=-68.0)
    try:
        axis.dist = 8.5
    except AttributeError:
        pass
    axis.legend(loc='upper left')
    status = axis.text2D(0.99, 0.96, '', transform=axis.transAxes,
                         ha='right', va='top', fontsize=10,
                         bbox={'facecolor': 'white', 'alpha': 0.8,
                               'edgecolor': 'gray'})
    paused = [False]

    def on_key(event):
        if event.key == ' ':
            paused[0] = not paused[0]
        elif event.key in ('q', 'escape'):
            plt.close(figure)

    def update(_frame):
        if not paused[0]:
            environment.step(interval_ms / 1000.0 * speed)
        for obstacle, collection in moving_artists:
            collection.set_verts(box_faces(obstacle))
        state = 'PAUSED' if paused[0] else 'RUNNING'
        status.set_text('{}   t={:.1f} s   speed={}x\nSpace: pause   Q: quit'.format(
            state, environment.time, speed))
        return [collection for _, collection in moving_artists] + [status]

    figure.canvas.mpl_connect('key_press_event', on_key)
    animation = FuncAnimation(figure, update, interval=interval_ms, blit=False,
                              cache_frame_data=False)
    figure._subject3_animation = animation
    figure.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Animate the subject-3 field in 3-D.')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='simulation speed multiplier (default: 1)')
    parser.add_argument('--interval', type=int, default=50,
                        help='display update interval in milliseconds (default: 50)')
    parser.add_argument('--vertical-exaggeration', type=float, default=4.0,
                        help='visual altitude exaggeration only (default: 4)')
    args = parser.parse_args()
    if args.speed <= 0.0 or args.interval <= 0 or args.vertical_exaggeration <= 0.0:
        parser.error('--speed, --interval and --vertical-exaggeration must be positive')
    run_animation(args.speed, args.interval, args.vertical_exaggeration)


if __name__ == '__main__':
    main()
