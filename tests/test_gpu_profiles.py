"""GPU search tuning must preserve CPU fallback and physical constraints."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gpu_profile', ROOT/'scripts/gpu_profile.py')
gpu_profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gpu_profile)
SOURCE = ROOT/'src/duojin01_bringup/config/nav2.yaml'


@pytest.mark.parametrize('preset', ['baseline', 'coverage', 'explore-lite', 'explore'])
def test_native_fallback_and_hardware_are_preserved(preset):
    original = yaml.safe_load(SOURCE.read_text())
    candidate = gpu_profile.make_profile(SOURCE, preset=preset)
    follow = candidate['controller_server']['ros__parameters']['FollowPath']
    for key in list(follow):
        if key.startswith('gpu_') or key in ('compute_backend', 'sampling_backend'):
            del follow[key]
    assert candidate == original


@pytest.mark.parametrize('overrides', [
    {'gpu_batch_size': 32768, 'gpu_time_steps': 56},
    {'gpu_batch_size': -1}, {'gpu_vy_std': float('nan')},
    {'gpu_path_weight': -1.}, {'gpu_temperature': 0.},
    {'gpu_time_steps': 80}, {'vx_max': .8},
])
def test_invalid_or_unobserved_search_is_rejected(overrides):
    with pytest.raises(ValueError):
        gpu_profile.make_profile(SOURCE, preset='explore-lite', overrides=overrides)


def test_support_cannot_exceed_horizon():
    with pytest.raises(ValueError):
        gpu_profile.make_profile(SOURCE, support_points=100)
