#!/usr/bin/env python3
"""Generate an explicit optional-GPU profile from the primary navigation YAML."""
import argparse
import json
import math
from pathlib import Path
import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/duojin01_bringup'))
from duojin01_bringup.chassis_profile import component_file, default_profile

EXPLORATION = dict(gpu_batch_size=6400, gpu_support_points=4,
                   gpu_vx_std=.35, gpu_vy_std=.35, gpu_wz_std=.12,
                   gpu_path_weight=1.5, gpu_goal_weight=24.)

# GPU-only overrides keep native MPPI's CPU fallback and hardware limits intact.
PRESETS = {
    'baseline': {},
    'coverage': {'gpu_batch_size': 6400},
    'explore': EXPLORATION,
    'explore-lite': {**EXPLORATION, 'gpu_batch_size': 3200},
    'structured': {**EXPLORATION, 'gpu_batch_size': 1600,
                   'gpu_candidate_mode': 'path_primitives', 'gpu_primitive_fraction': .25},
}


def make_profile(source, compute_backend='auto', sampling_backend='auto',
                 deadline_ms=40., minimum_work=262144, support_points=None,
                 preset='baseline', overrides=None):
    if compute_backend not in ('cpu', 'auto', 'cuda'):
        raise ValueError('Invalid compute backend')
    if sampling_backend not in ('original', 'auto', 'cuda'):
        raise ValueError('Invalid sampling backend')
    if not math.isfinite(deadline_ms) or not 0 < deadline_ms <= 50:
        raise ValueError('Deadline must be in (0, 50] ms')
    if minimum_work < 0 or (support_points is not None and support_points < 1):
        raise ValueError('Invalid workload/support point limits')
    if preset not in PRESETS:
        raise ValueError('Invalid GPU preset')
    if sampling_backend == 'original' and (preset != 'baseline' or overrides):
        raise ValueError('Sampling overrides require auto or cuda sampling backend')
    data = yaml.safe_load(Path(source).read_text())
    controller = data['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    if follow['plugin'] != 'duojin01_navigation::PreciseMPPI':
        raise ValueError('This profile requires PreciseMPPI')
    if sampling_backend != 'original' and follow['motion_model'] != 'Omni':
        raise ValueError('The optional sampler supports the Omni model')
    if deadline_ms > 1000. / controller['controller_frequency']:
        raise ValueError('GPU deadline exceeds the controller period')
    follow.update(compute_backend=compute_backend, sampling_backend=sampling_backend,
                  gpu_deadline_ms=deadline_ms, gpu_min_work=minimum_work,
                  gpu_support_points=8)
    follow.update(PRESETS[preset])
    if support_points is not None:
        follow['gpu_support_points'] = support_points
    allowed = {'gpu_batch_size', 'gpu_time_steps', 'gpu_vx_std', 'gpu_vy_std',
               'gpu_wz_std', 'gpu_path_weight', 'gpu_goal_weight', 'gpu_temperature',
               'gpu_candidate_mode', 'gpu_primitive_fraction'}
    if overrides:
        if set(overrides) - allowed:
            raise ValueError('Unsupported GPU override')
        follow.update(overrides)
    if follow.get('gpu_candidate_mode', 'noise') not in ('noise', 'path_primitives'):
        raise ValueError('Invalid GPU candidate mode')
    fraction = follow.get('gpu_primitive_fraction', .25)
    if not math.isfinite(fraction) or not 0 < fraction <= 1:
        raise ValueError('GPU primitive fraction must be in (0, 1]')
    batch = follow.get('gpu_batch_size', follow['batch_size'])
    steps = follow.get('gpu_time_steps', follow['time_steps'])
    support = follow['gpu_support_points']
    if any(type(v) is not int for v in (batch, steps, support)) or not (
            2 <= batch <= 32768 and 1 <= steps <= 512 and
            batch*steps <= 1048576 and 1 <= support <= steps):
        raise ValueError('Invalid GPU sampling dimensions or total state budget')
    for key, fallback in [('gpu_vx_std', 'vx_std'), ('gpu_vy_std', 'vy_std'),
                          ('gpu_wz_std', 'wz_std'), ('gpu_temperature', 'temperature')]:
        value = follow.get(key, follow[fallback])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    for key in ('gpu_path_weight', 'gpu_goal_weight', 'gpu_yaw_weight', 'gpu_cost_weight'):
        if key in follow and (not math.isfinite(follow[key]) or follow[key] < 0):
            raise ValueError(f'{key} must be finite and nonnegative')
    # A longer horizon needs enough observed map for the entire Omni footprint.
    # Unchanged baseline profiles remain compatible with their source settings.
    if 'gpu_time_steps' in (overrides or {}):
        local = data['local_costmap']['local_costmap']['ros__parameters']
        footprint = json.loads(local['footprint'])
        padding = local.get('footprint_padding', 0.)
        radius = max(math.hypot(abs(x)+padding, abs(y)+padding) for x, y in footprint)
        speed = math.hypot(max(follow['vx_max'], -follow['vx_min']), follow['vy_max'])
        travel = steps*follow['model_dt']*speed + radius + .012
        if travel >= min(local['width'], local['height'])/2:
            raise ValueError('GPU horizon exceeds the local map envelope at configured Omni limits')
    return data


def main():
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chassis-profile', type=Path, default=Path(default_profile()))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compute-backend', choices=['cpu', 'auto', 'cuda'], default='auto')
    parser.add_argument('--sampling-backend', choices=['original', 'auto', 'cuda'], default='auto')
    parser.add_argument('--deadline-ms', type=float, default=40.)
    parser.add_argument('--minimum-work', type=int, default=262144)
    parser.add_argument('--support-points', type=int)
    parser.add_argument('--preset', choices=list(PRESETS), default='baseline')
    for key, kind in [('batch-size', int), ('time-steps', int), ('vx-std', float),
                      ('vy-std', float), ('wz-std', float), ('temperature', float)]:
        parser.add_argument('--gpu-'+key, type=kind)
    parser.add_argument('--path-weight', type=float)
    parser.add_argument('--goal-weight', type=float)
    args = parser.parse_args()
    if args.chassis_profile.resolve() == args.output.resolve():
        parser.error('Output must differ from the primary input profile')
    overrides = {key: getattr(args, key) for key in ('gpu_batch_size', 'gpu_time_steps',
                 'gpu_vx_std', 'gpu_vy_std', 'gpu_wz_std', 'gpu_temperature')
                 if getattr(args, key) is not None}
    overrides.update({f'gpu_{key}': getattr(args, key) for key in ('path_weight', 'goal_weight')
                      if getattr(args, key) is not None})
    try:
        data = make_profile(component_file('navigation', args.chassis_profile), args.compute_backend, args.sampling_backend,
                            args.deadline_ms, args.minimum_work, args.support_points,
                            args.preset, overrides)
    except ValueError as e:
        parser.error(str(e))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(data, sort_keys=False))
    print(args.output.resolve())


if __name__ == '__main__':
    main()
