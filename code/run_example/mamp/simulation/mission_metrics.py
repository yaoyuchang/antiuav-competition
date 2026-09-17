"""Energy and path-smoothness metrics for the official 5-item scoring rubric.

These two metrics (集群总能量消耗 20%, 路径平滑性 10%) were previously
uncomputable because the evaluator only recorded a coarse, per-cycle-boundary
snapshot of each UAV's state.  ``FinalValidationEvaluator.run()`` now also
keeps the fine-grained (``EXECUTION_RECORD_DT``-resolution) position/velocity/
acceleration samples for every executed 0.2 s segment in each cycle log entry
(``segment_positions``/``segment_velocities``/``segment_accelerations``/
``segment_times``); this module integrates those samples.

Definitions used here (documented so a judge can check them against the
official statement):

- **Energy** -- the statement defines it as ``Sigma integral(|required
  overload|) dt`` summed over the swarm.  "Required overload" is read as the
  acceleration magnitude expressed in g, i.e. ``|a(t)| / G``, using the same
  gravity constant (``subject3_config.G``) the primitive-selection validator
  already uses to normalise accelerations.  Each UAV's overload trace is
  integrated (trapezoidal rule) over mission time and the 24 per-UAV energies
  are summed for the swarm total.
- **Smoothness** -- the statement defines it as ``integral(curvature^2) ds``.
  Curvature is computed from the recorded velocity/acceleration samples as
  ``kappa = |v x a| / |v|^3`` (guarded against near-zero speed), and
  ``ds = |v| dt``, so ``integral(kappa^2) ds`` reduces to
  ``integral(|v x a|^2 / |v|^5) dt``.  The statement does not say whether this
  is a per-path value averaged over the swarm or a summed "total" the way
  energy explicitly is ("集群总能量消耗" says "total"; "路径平滑性" does not).
  This module reports both: ``mean_path_smoothness`` (used as the scored
  value) and ``total_path_smoothness`` (sum over all 24 UAVs, for reference).
"""

import numpy as np

from ..configs import subject3_config as config


def _integrate_uav_energy(velocities, accelerations, times):
    """Trapezoidally integrate |a(t)|/G over one UAV's full mission."""
    overload = np.linalg.norm(accelerations, axis=1) / config.G
    # np.trapz (not np.trapezoid) for compatibility with the pinned NumPy
    # 1.24.4 / Python 3.8 runtime this project targets.
    return float(np.trapz(overload, times))


def _integrate_uav_smoothness(velocities, accelerations, times):
    """Trapezoidally integrate curvature^2 * |v| over one UAV's full mission."""
    speed = np.linalg.norm(velocities, axis=1)
    cross = np.linalg.norm(np.cross(velocities, accelerations), axis=1)
    safe_speed = np.maximum(speed, 1.0e-6)
    curvature = cross / safe_speed ** 3
    integrand = curvature ** 2 * speed
    return float(np.trapz(integrand, times))


def compute_energy_and_smoothness(cycle_logs):
    """Integrate energy and smoothness over the whole recorded mission.

    Args:
        cycle_logs: ``FinalScenarioResult.cycle_logs`` -- a tuple of per-cycle
            dicts, each carrying ``segment_velocities``, ``segment_accelerations``
            and ``segment_times`` arrays of shape
            ``(num_uav, len(execution_times))`` / ``(len(execution_times),)``.

    Returns:
        dict with ``total_energy``, ``per_uav_energy`` (list),
        ``mean_path_smoothness``, ``total_path_smoothness``,
        ``per_uav_smoothness`` (list).
    """
    if not cycle_logs:
        return {
            'total_energy': 0.0, 'per_uav_energy': [],
            'mean_path_smoothness': 0.0, 'total_path_smoothness': 0.0,
            'per_uav_smoothness': [],
        }

    num_uav = cycle_logs[0]['segment_velocities'].shape[0]
    per_uav_energy = np.zeros(num_uav)
    per_uav_smoothness = np.zeros(num_uav)

    for log in cycle_logs:
        times = log['segment_times']
        for uav in range(num_uav):
            velocities = log['segment_velocities'][uav]
            accelerations = log['segment_accelerations'][uav]
            per_uav_energy[uav] += _integrate_uav_energy(
                velocities, accelerations, times)
            per_uav_smoothness[uav] += _integrate_uav_smoothness(
                velocities, accelerations, times)

    return {
        'total_energy': float(np.sum(per_uav_energy)),
        'per_uav_energy': per_uav_energy.tolist(),
        'mean_path_smoothness': float(np.mean(per_uav_smoothness)),
        'total_path_smoothness': float(np.sum(per_uav_smoothness)),
        'per_uav_smoothness': per_uav_smoothness.tolist(),
    }
