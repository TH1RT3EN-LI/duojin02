"""GPU search tuning must preserve CPU fallback and physical constraints."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gpu_profile', ROOT/'scripts/gpu_profile.py')
gpu_profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gpu_profile)
from conftest import CHASSIS_PROFILE
from duojin01_bringup.chassis_profile import component_file


def navigation_source():
    return Path(component_file('navigation', str(CHASSIS_PROFILE)))


@pytest.mark.parametrize('preset', ['baseline', 'coverage', 'explore-lite', 'explore', 'structured'])
def test_native_fallback_and_hardware_are_preserved(preset):
    source = navigation_source()
    original = yaml.safe_load(source.read_text())
    candidate = gpu_profile.make_profile(source, preset=preset)
    for document in (original, candidate):
        follow = document['controller_server']['ros__parameters']['FollowPath']
        for key in list(follow):
            if key.startswith('gpu_') or key in ('compute_backend', 'sampling_backend'):
                del follow[key]
    assert candidate == original


@pytest.mark.parametrize('overrides', [
    {'gpu_batch_size': 32768, 'gpu_time_steps': 56},
    {'gpu_batch_size': -1}, {'gpu_vy_std': float('nan')},
    {'gpu_path_weight': -1.}, {'gpu_temperature': 0.},
    {'gpu_time_steps': 80}, {'vx_max': .8},
    {'gpu_candidate_mode': 'unknown'}, {'gpu_primitive_fraction': 0.},
    {'gpu_primitive_fraction': float('nan')}, {'gpu_primitive_fraction': 1.1},
])
def test_invalid_or_unobserved_search_is_rejected(overrides):
    with pytest.raises(ValueError):
        gpu_profile.make_profile(navigation_source(), preset='explore-lite', overrides=overrides)


def test_support_cannot_exceed_horizon():
    with pytest.raises(ValueError):
        gpu_profile.make_profile(navigation_source(), support_points=100)
