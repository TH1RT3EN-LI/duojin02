"""Startup checks one current observation without a confirmation state machine."""
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as S

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/duojin02_bringup'))
from duojin02_bringup.localization_readiness import ScanReadiness
from duojin02_bringup import safety_profile
from duojin02_bringup.safety_profile import safety_defaults


def scan(t, frame='laser', ranges=None):
    ns = round(t * 1e9)
    return S(header=S(stamp=S(sec=ns // 10**9, nanosec=ns % 10**9), frame_id=frame),
             angle_min=-1., angle_max=1., angle_increment=1., range_min=.3, range_max=8.,
             scan_time=.1, time_increment=0.,
             ranges=[1., 2., math.inf] if ranges is None else ranges)


def test_first_current_scan_starts_when_its_transform_is_available():
    state = ScanReadiness()
    state.receive(scan(1.), 10**9, 1.)
    assert state.poll(10**9, 1., lambda m: True)


def test_missing_exact_transform_can_only_hold_candidate_until_shared_timeout():
    state = ScanReadiness()
    next_time = 1. + state.timeout_s + .01
    old, latest = scan(1.), scan(next_time)
    state.receive(old, 10**9, 1.)
    assert not state.poll(1_020_000_000, 1.02, lambda m: False)
    state.receive(scan(1.1), 1_100_000_000, 1.1)
    assert not state.poll(1_120_000_000, 1.12, lambda m: m is latest)
    state.receive(latest, round(next_time * 1e9), next_time)
    assert state.poll(round((next_time + .02) * 1e9), next_time + .02, lambda m: m is latest)


def test_continuous_scans_do_not_starve_when_transform_lag_exceeds_scan_period():
    state = ScanReadiness()
    ready = False
    for step in range(16):
        now = 1. + step * .02
        if step % 5 == 0:  # 10 Hz laser, with exact TF published 0.12 s later.
            state.receive(scan(now), round(now * 1e9), now)
        ready = state.poll(round(now * 1e9), now,
                           lambda m: round(now * 1e9) - state.stamp_ns(m) >= 120_000_000)
        if ready:
            break
    assert ready
    assert now < 1. + state.timeout_s


def test_delayed_transform_can_qualify_pending_scan_without_more_callbacks():
    state = ScanReadiness()
    state.receive(scan(1.), 10**9, 1.)
    assert not state.poll(1_020_000_000, 1.02, lambda m: False)
    assert state.poll(1_040_000_000, 1.04, lambda m: True)


def test_duplicates_and_old_scans_do_not_refresh_receipt_during_paused_ros_time():
    state = ScanReadiness()
    limit = state.timeout_s
    state.receive(scan(1.), 10**9, 1.)
    state.receive(scan(1.), 10**9, 1. + limit * .9)
    state.receive(scan(.99), 10**9, 1. + limit * .9)
    assert not state.poll(10**9, 1. + limit + .001, lambda m: True)
    # Expiry cannot turn another duplicate callback into fresh evidence.
    state.receive(scan(1.), 10**9, 1. + limit + .01)
    assert not state.poll(10**9, 1. + limit + .01, lambda m: True)


@pytest.mark.parametrize('timeout_s', [.15, .6])
def test_source_and_receipt_freshness_both_follow_selected_timeout(timeout_s):
    state = ScanReadiness(timeout_s)
    state.receive(scan(1.), 10**9, 10.)
    assert state.poll(round((1. + timeout_s * .9) * 1e9),
                      10. + timeout_s * .9, lambda m: True)
    assert not state.poll(round((1. + timeout_s + .001) * 1e9),
                          10. + timeout_s * .9, lambda m: True)
    state.receive(scan(2.), 2 * 10**9, 20.)
    assert not state.poll(2 * 10**9, 20. + timeout_s + .001, lambda m: True)


def test_ros_clock_rewind_discards_old_scan_and_accepts_current_epoch():
    state = ScanReadiness()
    state.receive(scan(2.), 2_000_000_000, 1.)
    assert not state.poll(1_000_000_000, 1.02, lambda m: True)
    state.receive(scan(1.), 1_000_000_000, 1.03)
    assert state.poll(1_020_000_000, 1.05, lambda m: True)


def test_malformed_scan_stamp_metadata_or_wholly_unusable_ranges_blocks_startup():
    invalids = []
    m = scan(1.1); m.header.stamp.nanosec = 10**9; invalids.append(m)
    m = scan(1.1); m.header.stamp.sec = -1; invalids.append(m)
    m = scan(1.1); m.angle_max = 2.; invalids.append(m)
    m = scan(1.1); m.angle_increment = math.nan; invalids.append(m)
    invalids += [scan(1.1, frame=''), scan(1.1, ranges=[]),
                 scan(1.1, ranges=[math.nan] * 3),
                 scan(1.1, ranges=[math.inf, -math.inf, 1.]),
                 scan(1.1, ranges=[.1] * 3), scan(0.)]
    for malformed in invalids:
        state = ScanReadiness()
        state.receive(scan(1.), 10**9, 1.)
        state.receive(malformed, 1_100_000_000, 1.1)
        assert not state.poll(1_120_000_000, 1.12, lambda m: True)


def test_positive_inf_and_partial_nan_can_qualify_scan_liveness():
    state = ScanReadiness()
    state.receive(scan(1., ranges=[math.nan, math.inf, 1.]), 10**9, 1.)
    assert state.poll(1_020_000_000, 1.02, lambda m: True)


def test_invalid_clock_or_transform_error_never_starts_navigation():
    state = ScanReadiness()
    state.receive(scan(1.), 10**9, 1.)
    def unavailable(_):
        raise RuntimeError('TF publication not ready')
    assert not state.poll(1_020_000_000, 1.02, unavailable)
    assert not state.poll(0, 1.03, lambda m: True)
    state.receive(scan(1.1), 1_100_000_000, math.nan)
    assert not state.poll(1_120_000_000, 1.12, lambda m: True)


def test_startup_uses_guard_configuration_as_its_only_default_source():
    config = Path(__file__).resolve().parents[1] / 'src/duojin02_bringup/config/safety.yaml'
    data = yaml.safe_load(config.read_text())
    expected = {**data['/**']['ros__parameters'],
                **data['velocity_guard']['ros__parameters']}
    assert safety_defaults() == expected
    assert ScanReadiness().timeout_s == expected['scan_timeout_s']


def test_installed_safety_profile_reads_package_share_without_source_tree(tmp_path, monkeypatch):
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    data = {'/**': {'ros__parameters': {'command_timeout_s': .41}},
            'velocity_guard': {'ros__parameters': {'scan_timeout_s': .52,
                                                   'localization_tf_timeout_s': .63}}}
    (share / 'config/safety.yaml').write_text(yaml.safe_dump(data))
    monkeypatch.setattr(safety_profile, '__file__',
                        str(tmp_path / 'lib/python/duojin02_bringup/safety_profile.py'))
    packages = ModuleType('ament_index_python.packages')
    requested = []
    def get_share(name):
        requested.append(name)
        return str(share)
    packages.get_package_share_directory = get_share
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    assert safety_defaults() == {'command_timeout_s': .41, 'scan_timeout_s': .52,
                                 'localization_tf_timeout_s': .63}
    assert requested == ['duojin02_bringup']


def test_mux_uses_shared_lease_for_every_source_without_rewriting_config(tmp_path, monkeypatch):
    source = tmp_path / 'twist_mux.yaml'
    topics = {'navigation': {'topic': '/cmd_vel', 'priority': 80},
              'joy': {'topic': '/cmd_vel_joy', 'priority': 60}}
    source.write_text(yaml.safe_dump({'twist_mux': {'ros__parameters': {'topics': topics}}}))
    original = source.read_bytes()
    monkeypatch.setattr(safety_profile, 'safety_defaults', lambda: {'command_timeout_s': .41})
    effective = safety_profile.mux_parameters(source)
    assert effective['topics'] == {name: {**value, 'timeout': .41}
                                   for name, value in topics.items()}
    assert source.read_bytes() == original
