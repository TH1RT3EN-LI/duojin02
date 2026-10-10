#!/usr/bin/env python3
"""Generate an explicit optional-GPU profile from the primary navigation YAML."""
import argparse
import math
from pathlib import Path
import yaml


def make_profile(source, compute_backend='auto', sampling_backend='auto',
                 deadline_ms=40., minimum_work=262144, support_points=8):
    if compute_backend not in ('cpu', 'auto', 'cuda'):
        raise ValueError('Invalid compute backend')
    if sampling_backend not in ('original', 'auto', 'cuda'):
        raise ValueError('Invalid sampling backend')
    if not math.isfinite(deadline_ms) or not 0 < deadline_ms <= 50:
        raise ValueError('Deadline must be in (0, 50] ms')
    if minimum_work < 0 or support_points < 1:
        raise ValueError('Invalid workload/support point limits')
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
                  gpu_support_points=support_points)
    return data


def main():
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root/'src/duojin01_bringup/config/nav2.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compute-backend', choices=['cpu', 'auto', 'cuda'], default='auto')
    parser.add_argument('--sampling-backend', choices=['original', 'auto', 'cuda'], default='auto')
    parser.add_argument('--deadline-ms', type=float, default=40.)
    parser.add_argument('--minimum-work', type=int, default=262144)
    parser.add_argument('--support-points', type=int, default=8)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error('Output must differ from the primary input profile')
    data = make_profile(args.source, args.compute_backend, args.sampling_backend,
                        args.deadline_ms, args.minimum_work, args.support_points)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(data, sort_keys=False))
    print(args.output.resolve())


if __name__ == '__main__':
    main()
