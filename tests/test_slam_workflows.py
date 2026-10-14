"""Failure and ownership contracts for saved maps and recorded-data SLAM."""

import importlib.util
import json
import sys
import time
from pathlib import Path

import pytest
from geometry_msgs.msg import TransformStamped
from launch import LaunchContext
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/duojin02_slam_tools'))
from duojin02_slam_tools.snapshots import artifact_path, commit_snapshot, reserve_snapshot, validate_map
from duojin02_slam_tools.replay_tf import retained


def write_staged(prefix, graph=True):
    artifact_path(prefix, '.yaml').write_text(f'image: {prefix.name}.pgm\nresolution: 0.03\norigin: [0, 0, 0]\n')
    artifact_path(prefix, '.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    if graph:
        artifact_path(prefix, '.posegraph').write_bytes(b'graph')
        artifact_path(prefix, '.data').write_bytes(b'scans')


def test_complete_snapshot_detects_later_damage_and_never_overwrites(tmp_path):
    with reserve_snapshot(tmp_path, 'room.v2') as (pending, final):
        write_staged(pending)
        saved = commit_snapshot(pending, final, {'test': True})
    assert saved.name == 'room.v2.yaml'
    assert validate_map(saved)[1].name == 'room.v2.pgm'
    manifest = json.loads((tmp_path / 'room.v2.snapshot.json').read_text())
    assert set(manifest['files']) == {'image', 'yaml', 'posegraph', 'data'}
    with pytest.raises(FileExistsError):
        with reserve_snapshot(tmp_path, 'room.v2'):
            pass
    (tmp_path / 'room.v2.data').write_bytes(b'damaged')
    with pytest.raises(ValueError, match='modified'):
        validate_map(saved)


def test_failed_graph_never_publishes_a_loadable_map(tmp_path):
    with reserve_snapshot(tmp_path, 'auto') as (pending, final):
        write_staged(pending, graph=False)
        with pytest.raises(ValueError, match='complete'):
            commit_snapshot(pending, final, {})
    assert not list(tmp_path.glob('*.yaml'))
    assert not list(tmp_path.glob('*.pgm'))


def test_io_failure_rolls_back_partial_publication(tmp_path, monkeypatch):
    original = Path.replace
    def fail_manifest(path, target):
        if path.name.endswith('.snapshot.json'):
            raise OSError('disk write failed')
        return original(path, target)
    monkeypatch.setattr(Path, 'replace', fail_manifest)
    with reserve_snapshot(tmp_path, 'auto') as (pending, final):
        write_staged(pending)
        with pytest.raises(OSError):
            commit_snapshot(pending, final, {})
    assert not [p for p in tmp_path.iterdir() if p.name != '.save-map.lock']


def test_snapshot_lock_bounds_concurrent_writers(tmp_path):
    with reserve_snapshot(tmp_path, 'auto'):
        with pytest.raises(TimeoutError):
            with reserve_snapshot(tmp_path, 'auto', deadline=time.monotonic() + 0.05):
                pass


def test_auto_names_skip_orphaned_previous_artifacts(tmp_path):
    (tmp_path / '12.data').write_bytes(b'incomplete')
    with reserve_snapshot(tmp_path, 'auto') as (_, final):
        assert final.name == '13'


def transform(parent, child):
    value = TransformStamped()
    value.header.frame_id = parent
    value.child_frame_id = child
    value.transform.rotation.w = 1.0
    return value


def test_replay_tf_has_one_owner_for_map_and_odometry():
    frames = [transform('/map', 'odom'), transform('odom', 'base_footprint'),
              transform('base_footprint', 'base_link'), transform('base_link', 'laser')]
    assert [item.child_frame_id for item in retained(frames, 'recorded')] == [
        'base_footprint', 'base_link', 'laser']
    assert [item.child_frame_id for item in retained(frames, 'ekf')] == ['base_link', 'laser']


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'src/duojin02_bringup/launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('route', ['recorded', 'ekf'])
def test_offline_launch_starts_only_recorded_data_consumers(route):
    context = LaunchContext()
    context.launch_configurations.update(odom_source=route, slam_params_file='/tmp/config.yaml', scan_queue_size='100', scan_start_time_ns='0')
    nodes = load_launch('offline_mapping').create_nodes(context)
    assert all(isinstance(node, Node) for node in nodes)
    assert len(nodes) == (2 if route == 'recorded' else 3)
    assert {node.node_package for node in nodes} <= {'slam_toolbox', 'duojin02_slam_tools', 'robot_localization'}


def test_continuation_requires_a_real_graph_and_explicit_pose(tmp_path):
    module = load_launch('slam')
    context = LaunchContext()
    context.launch_configurations.update(pose_graph=str(tmp_path / 'room'), map_start_pose='', start_at_dock='false', slam_params_file='/tmp/config.yaml')
    with pytest.raises(ValueError, match='Missing pose graph'):
        module.create_node(context)
    (tmp_path / 'room.posegraph').write_bytes(b'graph')
    (tmp_path / 'room.data').write_bytes(b'data')
    with pytest.raises(ValueError, match='specify either'):
        module.create_node(context)
    context.launch_configurations['map_start_pose'] = '[1, 2, 0.5]'
    assert len(module.create_node(context)) == 1
