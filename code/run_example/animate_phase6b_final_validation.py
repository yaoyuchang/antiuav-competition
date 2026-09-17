"""Run and animate one 24-UAV Subject-3 final-validation scenario."""

import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from animate_subject3_environment_3d import box_faces, display_points, draw_cylinder
from run_phase6b_final_validation import formal_case
from mamp.configs import subject3_config as config


def _scenario(mode, seed):
    evaluator, starts, goals = formal_case(
        'switch' if mode == 'competition' else mode, seed)
    if mode == 'competition':
        # Combine the separately validated switch and noisy-target behaviours
        # for the closest visual reproduction of the written task.
        for manager in evaluator.goal_managers:
            manager.dynamic_model.sinusoid_enabled = True
            manager.dynamic_model.random_enabled = True
    # Wall-clock deadline is an evaluation metric, not a flight constraint.
    # Rendering runs must continue through occasional host scheduling spikes.
    evaluator.cycle_deadline = float('inf')
    # formal_case owns the same obstacle instances used by every planner.
    return evaluator, starts, goals, evaluator.obstacles


def run_scenario(mode, seed, cycles):
    evaluator, starts, goals, obstacles = _scenario(mode, seed)
    print('planning {} cycles for mode={} ...'.format(cycles, mode), flush=True)
    result = evaluator.run('animation-{}'.format(mode), starts, goals, cycles,
                           stop_on_all_arrived=True)
    if not result.cycle_logs:
        raise RuntimeError('simulation produced no executable cycle: {}'.format(
            result.reason))

    positions = np.asarray([item['positions'] for item in result.cycle_logs])
    times = np.asarray([item['mission_time'] for item in result.cycle_logs])
    # Include the last executed state when the evaluator exposes it in a failure
    # snapshot.  Successful runs still have dense 0.2 s cycle-start samples.
    print('simulation success={} completed={}/{} mission_time={:.1f}s arrivals={}/24'.format(
        result.success, result.completed_cycles, cycles, result.mission_time,
        result.arrival_count), flush=True)
    if result.arrival_count == 24:
        print('all 24 UAVs arrived; planning stopped automatically', flush=True)
    elif result.completed_cycles == cycles:
        print('maximum cycle limit reached before all UAVs arrived', flush=True)
    print('minimum distance: UAV={:.3f}m static={:.3f}m dynamic={:.3f}m'.format(
        result.minimum_uav_distance, result.minimum_static_clearance,
        result.minimum_dynamic_clearance), flush=True)
    if not result.success:
        print('stopped early: {}'.format(result.reason), flush=True)
    return result, positions, times, obstacles, starts, goals


def animate(result, positions, times, obstacles, starts, initial_goals,
            interval_ms=40, playback_speed=5.0, overview=False,
            vertical_exaggeration=4.0, output=None, no_show=False):
    figure = plt.figure(figsize=(15, 8))
    axis = figure.add_subplot(111, projection='3d')
    moving_obstacles = []

    for obstacle in obstacles:
        obstacle.update(0.)
        color = '#5abf45' if obstacle.motion else '#2f91c2'
        if obstacle.shape == 'cylinder':
            draw_cylinder(axis, obstacle, color)
        else:
            collection = Poly3DCollection(
                box_faces(obstacle), facecolors=color, edgecolors='#263238',
                alpha=.45, linewidths=.45)
            axis.add_collection3d(collection)
            if obstacle.motion:
                moving_obstacles.append((obstacle, collection))

    start_display = display_points(starts)
    axis.scatter(start_display[:, 0], start_display[:, 1], start_display[:, 2],
                 s=13, color='black', alpha=.45, label='starts')
    initial_display = display_points(initial_goals)
    goal_artist = axis.scatter(
        initial_display[:, 0], initial_display[:, 1], initial_display[:, 2],
        marker='x', s=25, color='#e53935', label='current goals')

    colours = plt.cm.hsv(np.linspace(0., 1., 24, endpoint=False))
    lines, markers, labels = [], [], []
    for uav in range(24):
        line, = axis.plot([], [], [], color=colours[uav], linewidth=1., alpha=.8)
        marker, = axis.plot([], [], [], marker='o', linestyle='',
                            color=colours[uav], markersize=4)
        label = axis.text(0., 0., 0., str(uav + 1), fontsize=6)
        lines.append(line); markers.append(marker); labels.append(label)

    status = axis.text2D(.015, .975, '', transform=axis.transAxes,
                         va='top', bbox={'facecolor': 'white', 'alpha': .82})
    axis.set_xlabel('North x (m)')
    axis.set_ylabel('East z (m)')
    axis.set_zlabel('Up y (m)')
    axis.set_title('Subject 3: 24-UAV dynamic planning ({})'.format(result.label))
    axis.set_zlim(0., 110.)
    axis.view_init(elev=23., azim=-68.)
    # A literal 5000:1300:110 course ratio makes altitude almost invisible.
    # Square-root compression keeps the ordering of the physical dimensions
    # while producing a readable 3-D scene.  This affects display only.
    physical_ratio = np.array([5000., 1300., 110. * vertical_exaggeration])
    display_ratio = np.sqrt(physical_ratio)
    axis.set_box_aspect(display_ratio / display_ratio[2])
    axis.legend(loc='upper right')

    frame_step = max(1, int(round(
        interval_ms * playback_speed / (1000. * config.EXECUTION_HORIZON))))
    frame_indices = np.arange(0, len(times), frame_step, dtype=int)
    if frame_indices[-1] != len(times) - 1:
        frame_indices = np.append(frame_indices, len(times) - 1)

    def update(frame_number):
        index = int(frame_indices[frame_number])
        time_value = float(times[index])
        points = display_points(positions[index])
        for obstacle, collection in moving_obstacles:
            obstacle.update(time_value)
            collection.set_verts(box_faces(obstacle))
        for uav in range(24):
            history = display_points(positions[:index + 1, uav, :])
            lines[uav].set_data_3d(history[:, 0], history[:, 1], history[:, 2])
            markers[uav].set_data_3d([points[uav, 0]], [points[uav, 1]],
                                     [points[uav, 2]])
            labels[uav].set_position((points[uav, 0], points[uav, 1]))
            labels[uav].set_3d_properties(points[uav, 2] + 2.)
        goal_records = result.cycle_logs[index].get('goal_updates', ())
        if goal_records:
            shown_goals = display_points(np.asarray(
                [record.filtered_goal for record in goal_records]))
            goal_artist._offsets3d = (shown_goals[:, 0], shown_goals[:, 1],
                                      shown_goals[:, 2])
        minimum = result.cycle_logs[index]['minimum_pair_distance']
        status.set_text('t={:.1f}s  frame={}/{}  min UAV distance={:.2f}m'.format(
            time_value, frame_number + 1, len(frame_indices), minimum))
        if overview:
            axis.set_xlim(-100., 5100.); axis.set_ylim(-650., 650.)
        else:
            xmin, xmax = points[:, 0].min(), points[:, 0].max()
            view_xmin = max(-100., xmin - 120.)
            view_xmax = min(5100., xmax + 500.)
            east_min = points[:, 1].min() - 50.
            east_max = points[:, 1].max() + 50.
            # Keep nearby avoidance geometry in frame even when it is offset
            # laterally from the compact swarm formation.
            for obstacle in obstacles:
                if obstacle.x + obstacle.radius < view_xmin or \
                        obstacle.x - obstacle.radius > view_xmax:
                    continue
                if obstacle.shape in ('cube', 'rotated_cube'):
                    east_half = .5 * np.hypot(obstacle.length, obstacle.width)
                elif obstacle.shape == 'cylinder':
                    east_half = obstacle.cylinder_radius
                else:
                    east_half = obstacle.radius
                east_min = min(east_min, obstacle.z - east_half - 20.)
                east_max = max(east_max, obstacle.z + east_half + 20.)
            axis.set_xlim(view_xmin, view_xmax)
            axis.set_ylim(max(-650., east_min), min(650., east_max))
        return lines + markers + labels + [status, goal_artist]

    animation = FuncAnimation(figure, update, frames=len(frame_indices),
                              interval=interval_ms, blit=False,
                              cache_frame_data=False)
    figure._phase6b_animation = animation
    figure.tight_layout()
    if output:
        output = os.path.abspath(output)
        os.makedirs(os.path.dirname(output), exist_ok=True)
        print('saving animation to {} ...'.format(output), flush=True)
        animation.save(output, fps=max(1, int(round(1000. / interval_ms))))
        print('animation saved: {}'.format(output), flush=True)
    if not no_show:
        plt.show()
    else:
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description='Animate the 24-UAV Subject-3 final-validation simulation.')
    parser.add_argument('--mode',
                        choices=('competition', 'fixed', 'switch', 'sin', 'random'),
                        default='competition',
                        help='goal behaviour (default: competition = switch + noise)')
    parser.add_argument('--cycles', type=int, default=2000,
                        help='maximum cycles; stops when all UAVs arrive (default: 2000)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--speed', type=float, default=5.,
                        help='animation playback multiplier (default: 5)')
    parser.add_argument('--interval', type=int, default=40,
                        help='display interval in milliseconds (default: 40)')
    parser.add_argument('--overview', action='store_true')
    parser.add_argument('--vertical-exaggeration', type=float, default=4.)
    parser.add_argument('--output', help='optional .mp4 or .gif output path')
    parser.add_argument('--local-output',
                        help='save an additional local-follow animation')
    parser.add_argument('--overview-output',
                        help='save an additional full-course animation')
    parser.add_argument('--no-show', action='store_true',
                        help='do not open a window (normally used with --output)')
    args = parser.parse_args()
    if args.cycles <= 0 or args.speed <= 0 or args.interval <= 0:
        parser.error('--cycles, --speed and --interval must be positive')
    data = run_scenario(args.mode, args.seed, args.cycles)
    common = {'interval_ms': args.interval, 'playback_speed': args.speed,
              'vertical_exaggeration': args.vertical_exaggeration}
    if args.local_output or args.overview_output:
        if args.local_output:
            animate(*data, overview=False, output=args.local_output,
                    no_show=True, **common)
        if args.overview_output:
            animate(*data, overview=True, output=args.overview_output,
                    no_show=True, **common)
        if not args.no_show:
            animate(*data, overview=args.overview, output=None,
                    no_show=False, **common)
    else:
        animate(*data, overview=args.overview, output=args.output,
                no_show=args.no_show, **common)


if __name__ == '__main__':
    main()
