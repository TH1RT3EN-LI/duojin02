"""One authored profile owns motion, geometry, lidar and disposable ROS files."""

import copy
import json
import os
from pathlib import Path
import stat

import pytest
import yaml

from conftest import CHASSIS_PROFILE, ROOT
from duojin01_bringup import chassis_profile as config


@pytest.fixture
def master():
    return config.load_profile(CHASSIS_PROFILE)


def write_profile(path, profile):
    path.write_text(yaml.safe_dump(profile, sort_keys=False))
    return path


def test_current_selection_preserves_algorithm_precision_and_crop(master):
    navigation = config.render(master, 'navigation')
    follow = navigation['controller_server']['ros__parameters']['FollowPath']
    assert master['motion'] == dict(forward_mps=.5, reverse_mps=.5, lateral_mps=.5, yaw_radps=.65, narrow_mps=.5)
    assert follow['plugin'] == 'duojin01_navigation::PreciseMPPI'
    assert follow['compute_backend'] == 'cpu'
    assert follow['sampling_backend'] == 'cuda'
    assert follow['gpu_candidate_mode'] == 'path_primitives'
    assert follow['gpu_batch_size'] == 128
    assert follow['gpu_support_points'] == 4
    assert follow['gpu_deadline_ms'] == 40.0
    smoother = navigation['velocity_smoother']['ros__parameters']
    assert smoother['max_accel'] == [.5, .5, .8]
    assert smoother['max_decel'] == [-.4, -.4, -.8]
    guard = config.render(master, 'guard')['velocity_guard']['ros__parameters']
    assert guard['braking_deceleration_mps2'] == .4
    assert guard['clearance_margin_m'] == .008
    assert master['lidar']['min_range_m'] == .02
    assert master['lidar']['boundary_ambiguity_m'] == 0.0
    assert master['simulation']['lidar']['scan_height_m'] == .14330766582


@pytest.mark.parametrize('limits', [(.5, .5, .5), (.5, .2, .3), (.6, .4, .25)])
def test_three_independent_directions_update_controller_and_smoother_together(master, limits):
    forward, reverse, lateral = limits
    master['motion'].update(forward_mps=forward, reverse_mps=reverse, lateral_mps=lateral)
    before = copy.deepcopy(master)
    navigation = config.render(master, 'navigation')
    follow = navigation['controller_server']['ros__parameters']['FollowPath']
    smoother = navigation['velocity_smoother']['ros__parameters']
    assert (follow['vx_max'], follow['vx_min'], follow['vy_max']) == (forward, -reverse, lateral)
    assert smoother['max_velocity'] == [forward, lateral, master['motion']['yaw_radps']]
    assert smoother['min_velocity'] == [-reverse, -lateral, -master['motion']['yaw_radps']]
    assert master == before


def test_geometry_and_lidar_floor_propagate_to_all_consumers(master, tmp_path):
    polygon = [-.3, -.2, .3, -.2, .3, .2, -.3, .2]
    master['geometry']['body_polygon_xy_m'] = polygon
    master['lidar']['min_range_m'] = .035
    profile = write_profile(tmp_path/'chassis.yaml', master)
    navigation = config.render(master, 'navigation')
    for name in ('local_costmap', 'global_costmap'):
        assert json.loads(navigation[name][name]['ros__parameters']['footprint']) == [[-.3, -.2], [.3, -.2], [.3, .2], [-.3, .2]]
    assert navigation['amcl']['ros__parameters']['laser_min_range'] == .035
    assert config.render(master, 'guard')['velocity_guard']['ros__parameters']['footprint'] == polygon
    assert config.render(master, 'slam')['slam_toolbox']['ros__parameters']['min_laser_range'] == .035
    hardware = config.render(master, 'lidar')['lslidar_driver_node']['ros__parameters']
    simulation = config.simulation_profile(profile)['lidar']
    assert hardware['min_range'] == simulation['range_min_m'] == .035
    assert hardware['self_filter.body_polygon_xy_m'] == simulation['self_filter']['body_polygon_xy_m'] == polygon
    assert hardware['self_filter.body_z_bounds_m'] == simulation['self_filter']['body_z_bounds_m']
    assert hardware['self_filter.laser_to_body_xyz_m'] == simulation['self_filter']['laser_to_body_xyz_m']
    assert simulation['scan_height_m'] != simulation['self_filter']['laser_to_body_xyz_m'][2]


@pytest.mark.parametrize('section,key,value', [
    ('motion', 'forward_mps', 0), ('motion', 'reverse_mps', -.2),
    ('motion', 'lateral_mps', True), ('motion', 'yaw_radps', float('nan')),
    ('lidar', 'min_range_m', float('inf')), ('lidar', 'boundary_ambiguity_m', -.001),
    ('lidar', 'laser_to_body_xyz_m', [0., 0.]), ('lidar', 'crop_enabled', 'true'),
    ('geometry', 'body_z_bounds_m', [.2, .1]),
    ('geometry', 'body_polygon_xy_m', [0., 0., 0., 0., 1., 1.]),
])
def test_invalid_physical_inputs_fail_before_rendering(master, tmp_path, section, key, value):
    master[section][key] = value
    profile = write_profile(tmp_path/'chassis.yaml', master)
    with pytest.raises(config.ProfileError):
        config.load_profile(profile)


def test_schema_unknown_keys_and_nested_nonfinite_values_are_rejected(master, tmp_path):
    for mutation in ('schema', 'unknown', 'nested_nonfinite'):
        candidate = copy.deepcopy(master)
        if mutation == 'schema':
            candidate['schema_version'] = 0
        elif mutation == 'unknown':
            candidate['old_navigation_yaml'] = '/tmp/legacy.yaml'
        else:
            candidate['components']['guard']['velocity_guard']['ros__parameters']['range_error_m'] = float('nan')
        with pytest.raises(config.ProfileError):
            config.load_profile(write_profile(tmp_path/'chassis.yaml', candidate))


@pytest.mark.parametrize('target', ['motion', 'controller', 'smoother', 'lidar', 'footprint', 'guard', 'simulation'])
def test_reintroduced_or_conflicting_derived_fields_are_rejected(master, tmp_path, target):
    if target == 'motion':
        master['motion']['vx_max'] = .8
    elif target == 'controller':
        master['components']['navigation']['controller_server']['ros__parameters']['FollowPath']['vx_max'] = .8
    elif target == 'smoother':
        master['components']['navigation']['velocity_smoother']['ros__parameters']['max_velocity'] = [.8, .8, .8]
    elif target == 'lidar':
        master['lidar']['driver_parameters']['self_filter.body_polygon_xy_m'] = [-1., -1., 1., -1., 1., 1.]
    elif target == 'footprint':
        master['components']['navigation']['local_costmap']['local_costmap']['ros__parameters']['footprint'] = '[[-1,-1],[1,-1],[1,1]]'
    elif target == 'guard':
        master['components']['guard']['velocity_guard']['ros__parameters']['footprint'] = [-1., -1., 1., -1., 1., 1.]
    else:
        master['simulation']['lidar']['self_filter'] = {'enabled': False}
    with pytest.raises(config.ProfileError):
        config.load_profile(write_profile(tmp_path/'chassis.yaml', master))


@pytest.mark.parametrize('duplicate', ['schema_version: 1\n', '  forward_mps: 0.9\n'])
def test_duplicate_yaml_keys_are_rejected(tmp_path, duplicate):
    text = CHASSIS_PROFILE.read_text()
    if duplicate.startswith('  '):
        text = text.replace('motion:\n', 'motion:\n'+duplicate, 1)
    else:
        text += duplicate
    path = tmp_path/'chassis.yaml'
    path.write_text(text)
    with pytest.raises(config.ProfileError, match='duplicate'):
        config.load_profile(path)


def test_default_profile_respects_one_explicit_selection(master, tmp_path, monkeypatch):
    selected = write_profile(tmp_path/'chassis.yaml', master)
    monkeypatch.setenv('DUOJIN_CHASSIS_PROFILE', str(selected))
    assert config.default_profile() == str(selected.resolve())
    assert config.load_profile() == master


def test_generated_files_are_private_deterministic_and_follow_source_edits(master, tmp_path, monkeypatch):
    monkeypatch.setattr(config.tempfile, 'gettempdir', lambda: str(tmp_path))
    source = write_profile(tmp_path/'chassis.yaml', master)
    original_source = source.read_bytes()
    previous = None
    for component in sorted(config.COMPONENTS | {'lidar', 'slam_offline'}):
        path = Path(config.component_file(component, source))
        assert yaml.safe_load(path.read_text()) == config.render(master, component)
        assert config.component_file(component, source) == str(path)
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
        assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
        if component == 'navigation':
            previous = path
    assert source.read_bytes() == original_source
    old_payload = previous.read_bytes()
    master['motion']['reverse_mps'] = .2
    write_profile(source, master)
    changed = Path(config.component_file('navigation', source))
    assert changed != previous
    assert previous.read_bytes() == old_payload
    assert yaml.safe_load(changed.read_text())['velocity_smoother']['ros__parameters']['min_velocity'][0] == -.2


def test_generated_file_corruption_is_replaced_and_symlinks_are_refused(master, tmp_path, monkeypatch):
    monkeypatch.setattr(config.tempfile, 'gettempdir', lambda: str(tmp_path))
    source = write_profile(tmp_path/'chassis.yaml', master)
    path = Path(config.component_file('guard', source))
    expected = path.read_bytes()
    path.write_bytes(b'changed runtime file')
    assert Path(config.component_file('guard', source)).read_bytes() == expected
    path.unlink()
    external = tmp_path/'external.yaml'
    external.write_bytes(expected)
    path.symlink_to(external)
    with pytest.raises(config.ProfileError, match='symlink'):
        config.component_file('guard', source)
    assert external.read_bytes() == expected


def test_unsafe_runtime_directory_is_refused(master, tmp_path, monkeypatch):
    monkeypatch.setattr(config.tempfile, 'gettempdir', lambda: str(tmp_path))
    source = write_profile(tmp_path/'chassis.yaml', master)
    directory = tmp_path/f'duojin-chassis-{os.getuid()}'
    directory.mkdir(mode=0o755)
    with pytest.raises(config.ProfileError, match='unsafe'):
        config.component_file('guard', source)


def test_master_is_the_only_authored_chassis_yaml():
    authored = set((ROOT/'src/duojin01_bringup/config').rglob('*.yaml'))
    assert authored == {CHASSIS_PROFILE}
    assert not list((ROOT/'src/lslidar_driver/lslidar_driver/config').glob('*.yaml'))
    assert not list((ROOT/'src/duojin01_teleop/config').glob('*.yaml'))


def test_robot_laser_tf_and_crop_share_nonzero_master_extrinsic(master, tmp_path):
    import xml.etree.ElementTree as ET
    xacro = pytest.importorskip('xacro')
    xyz = [.12, -.035, .145]
    rpy = [.1, -.2, .35]
    master['lidar']['laser_to_body_xyz_m'] = xyz
    master['lidar']['laser_to_body_rpy_rad'] = rpy
    selected = write_profile(tmp_path/'chassis.yaml', master)
    source = ROOT/'src/duojin01_description/urdf/duojin01.xacro'
    original = ET.fromstring(xacro.process_file(str(source)).toxml())
    rendered = ET.fromstring(config.robot_description(selected, str(source)))
    optical = next(joint for joint in rendered.findall('joint') if joint.find('child').get('link') == 'laser')
    assert optical.get('type') == 'fixed'
    assert optical.find('parent').get('link') == 'base_link'
    assert list(map(float, optical.find('origin').get('xyz').split())) == xyz
    assert list(map(float, optical.find('origin').get('rpy').split())) == rpy
    crop = config.render(master, 'lidar')['lslidar_driver_node']['ros__parameters']
    assert crop['self_filter.laser_to_body_xyz_m'] == xyz
    assert crop['self_filter.laser_to_body_rpy_rad'] == rpy
    # Optical calibration changes only that frame, preserving measured visual geometry.
    for link in original.findall('link'):
        actual = rendered.find(f"link[@name='{link.get('name')}']")
        assert ET.tostring(actual) == ET.tostring(link)
    for joint in original.findall('joint'):
        if joint.find('child').get('link') != 'laser':
            actual = rendered.find(f"joint[@name='{joint.get('name')}']")
            assert ET.tostring(actual) == ET.tostring(joint)
    assert config.load_profile(selected) == master
