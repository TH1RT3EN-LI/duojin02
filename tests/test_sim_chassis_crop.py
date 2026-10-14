"""Simulator policy parity against the unchanged production C++ classifier."""
import ast
import copy
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
from conftest import CHASSIS_PROFILE, simulation_package
from duojin01_bringup.chassis_profile import load_profile, render, simulation_profile

SIM = simulation_package()
HEADER = ROOT / 'src/lslidar_driver/lslidar_driver/include'
SPEC = importlib.util.spec_from_file_location('sim_chassis_crop', SIM/'duojin02_sim/chassis_crop.py')
crop = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(crop)


def hardware_crop():
    return simulation_profile(CHASSIS_PROFILE)['lidar']['self_filter']


@pytest.fixture(scope='module')
def cpp_classifier(tmp_path_factory):
    if not shutil.which('g++'):
        pytest.skip('C++ compiler unavailable')
    directory = tmp_path_factory.mktemp('chassis-crop-parity')
    source, binary = directory/'classify.cpp', directory/'classify'
    source.write_text(r'''
#include "lslidar_driver/self_filter.hpp"
#include <iostream>
#include <string>
#include <vector>
double value() {std::string s; std::cin >> s; return std::stod(s);}
std::vector<double> vector() {
  size_t n; std::cin >> n; std::vector<double> out;
  for(size_t i=0;i<n;++i) out.push_back(value()); return out;
}
int main() {
  lslidar_driver::SelfFilterConfig c; int enabled; std::cin >> enabled;
  c.enabled=enabled; c.boundary_ambiguity_m=value();
  c.laser_to_body_xyz_m=vector();c.laser_to_body_rpy_rad=vector();
  c.body_polygon_xy_m=vector();c.body_z_bounds_m=vector();
  lslidar_driver::SelfFilter f(c);size_t n;std::cin >> n;
  for(size_t i=0;i<n;++i) {
    const lslidar_driver::Point3 p{value(),value(),value()};
    std::cout << static_cast<int>(f.classify(p)) << '\n';
  }
}
''')
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(HEADER), str(source), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


def native_classes(binary, config, points):
    values = [str(int(config['enabled'])), repr(config['boundary_ambiguity_m'])]
    for name in ('laser_to_body_xyz_m', 'laser_to_body_rpy_rad', 'body_polygon_xy_m', 'body_z_bounds_m'):
        vector = config[name]
        values += [str(len(vector)), *map(repr, vector)]
    values += [str(len(points))]
    values += [repr(v) for point in points for v in point]
    output = subprocess.run([str(binary)], input=' '.join(values), check=True,
                            capture_output=True, text=True).stdout
    return list(map(int, output.split()))


def assert_parity(binary, config, points):
    python = crop.ChassisCrop(config)
    expected = native_classes(binary, config, points)
    assert [python.classify(point) for point in points] == expected
    return expected


def test_hardware_and_simulation_crop_share_one_master_and_canonical_module():
    lidar = simulation_profile(CHASSIS_PROFILE)['lidar']
    parameters = render(load_profile(CHASSIS_PROFILE), 'lidar')['lslidar_driver_node']['ros__parameters']
    assert lidar['range_min_m'] == parameters['min_range'] == .02
    assert lidar['published_range_max_m'] == parameters['max_range']
    assert lidar['scan_height_m'] == .14330766582
    assert lidar['self_filter']['boundary_ambiguity_m'] == 0.0
    # Simulation height is an explicit override of the hardware mounting pose.
    assert lidar['self_filter']['laser_to_body_xyz_m'][2] == .1302326658155
    for name, value in lidar['self_filter'].items():
        assert value == parameters['self_filter.'+name]
    assert Path(crop.__file__).resolve() == (SIM/'duojin02_sim/chassis_crop.py').resolve()
    assert not (ROOT/'tests/performance/navigation_harness/sim_fixture').exists()


def effective_launch_crop(tmp_path, hardware_pose, simulation_pose, snapshot):
    """Run the launch's real pure configuration statements without importing ROS.

    The rest of start() builds node/process descriptions. We compile only its
    metadata reads, deep copy and both transform-validation calls, using their
    production AST rather than duplicating the effective-pose implementation.
    """
    path = SIM/'launch/lab.launch.py'
    module = ast.parse(path.read_text(), filename=str(path))
    start = next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == 'start')
    names = {'geometry', 'hardware_laser_pose', 'simulation_laser_pose', 'hardware_crop_config', 'crop_config'}
    statements = []
    for statement in start.body:
        if isinstance(statement, ast.Assign):
            targets = statement.targets
            if any(isinstance(target, ast.Name) and target.id in names or
                   isinstance(target, ast.Subscript) and isinstance(target.value, ast.Name) and target.value.id == 'crop_config'
                   for target in targets):
                statements.append(statement)
        elif isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call):
            if isinstance(statement.value.func, ast.Name) and statement.value.func.id == 'validate_laser_pose':
                statements.append(statement)
    assert sum(isinstance(node, ast.Expr) for node in statements) == 2
    assert sum(isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Subscript)
               for node in statements) == 2
    (tmp_path/'hardware-laser.json').write_text(json.dumps({
        'hardware_laser_xyz_rpy': hardware_pose, 'simulation_laser_xyz_rpy': simulation_pose}))
    scope = {'session': tmp_path, 'profile': {'lidar': {'self_filter': snapshot}},
             'json': json, 'copy': copy, 'validate_laser_pose': crop.validate_laser_pose}
    selected = ast.fix_missing_locations(ast.Module(body=statements, type_ignores=[]))
    exec(compile(selected, str(path), 'exec'), scope)
    return scope['crop_config']


@pytest.mark.parametrize('enabled', [True, False])
def test_launch_effective_scan_height_matches_sensor_without_mutating_crop_policy(tmp_path, enabled):
    snapshot = hardware_crop()
    snapshot['enabled'] = enabled
    before = copy.deepcopy(snapshot)
    hardware_pose = snapshot['laser_to_body_xyz_m']+snapshot['laser_to_body_rpy_rad']
    simulation_pose = list(hardware_pose)
    simulation_pose[2] = .14330766582
    effective = effective_launch_crop(tmp_path, hardware_pose, simulation_pose, snapshot)
    assert effective['laser_to_body_xyz_m'] == simulation_pose[:3]
    assert effective['laser_to_body_rpy_rad'] == hardware_pose[3:]
    assert effective['laser_to_body_xyz_m'][:2] == snapshot['laser_to_body_xyz_m'][:2]
    assert {key: value for key, value in effective.items() if key != 'laser_to_body_xyz_m'} == {
        key: value for key, value in snapshot.items() if key != 'laser_to_body_xyz_m'}
    assert snapshot == before
    assert effective is not snapshot
    assert effective['body_polygon_xy_m'] is not snapshot['body_polygon_xy_m']
    crop.validate_laser_pose(effective, simulation_pose)
    if enabled:
        with pytest.raises(ValueError, match='differs'):
            crop.validate_laser_pose(snapshot, simulation_pose)


def test_launch_validates_hardware_snapshot_before_adapting_simulation_pose(tmp_path):
    snapshot = hardware_crop()
    hardware_pose = snapshot['laser_to_body_xyz_m']+snapshot['laser_to_body_rpy_rad']
    simulation_pose = list(hardware_pose)
    simulation_pose[2] = .14330766582
    hardware_pose[0] += .001
    with pytest.raises(ValueError, match='differs'):
        effective_launch_crop(tmp_path, hardware_pose, simulation_pose, snapshot)


@pytest.mark.parametrize('height', [math.nan, math.inf, -math.inf])
def test_launch_refuses_nonfinite_effective_crop_height(tmp_path, height):
    snapshot = hardware_crop()
    hardware_pose = snapshot['laser_to_body_xyz_m']+snapshot['laser_to_body_rpy_rad']
    simulation_pose = list(hardware_pose)
    simulation_pose[2] = height
    with pytest.raises(ValueError):
        effective_launch_crop(tmp_path, hardware_pose, simulation_pose, snapshot)


def test_effective_height_crop_matches_cpp_and_preserves_external_boundary_policy(tmp_path, cpp_classifier):
    snapshot = hardware_crop()
    hardware_pose = snapshot['laser_to_body_xyz_m']+snapshot['laser_to_body_rpy_rad']
    simulation_pose = list(hardware_pose)
    simulation_pose[2] = .14330766582
    effective = effective_launch_crop(tmp_path, hardware_pose, simulation_pose, snapshot)
    # Sensor-plane points exercise the same projection policy at the two heights.
    points = [(0.05,0.,0.), (0.11,0.,0.), (0.,-.25,0.), (0.,.18,0.)]
    classes = assert_parity(cpp_classifier, effective, points)
    assert classes == assert_parity(cpp_classifier, snapshot, points)
    assert classes == [crop.ChassisCrop.INSIDE, crop.ChassisCrop.INSIDE,
                       crop.ChassisCrop.OUTSIDE, crop.ChassisCrop.INSIDE]


def test_cpp_python_boundary_concavity_and_tilted_height_parity(cpp_classifier):
    config = hardware_crop()
    # Exact boundary, both sides of the band, Z ambiguity and nonfinite points.
    xyz = config['laser_to_body_xyz_m']
    body = [(x,y,z) for x in (.213, .213-.03, .213-.030001, .213+.000001, -.214)
            for y in (0., .19, -.19) for z in (.1302326658155, .021, .021+.03, .021+.030001, .218)]
    points = [tuple(v-t for v,t in zip(p,xyz)) for p in body] + [(math.nan,0.,0.), (math.inf,0.,0.)]
    assert_parity(cpp_classifier, config, points)
    concave = dict(config, laser_to_body_xyz_m=[0.,0.,0.], laser_to_body_rpy_rad=[0.,0.,0.],
                   body_polygon_xy_m=[-2.,-2.,2.,-2.,2.,2.,.5,2.,.5,-.5,-.5,-.5,-.5,2.,-2.,2.],
                   body_z_bounds_m=[-.5,.5], boundary_ambiguity_m=.03)
    points = [(0.,1.,0.), (1.,1.,0.), (.5,1.,0.), (.500001,1.,0.), (.531,1.,0.)]
    assert assert_parity(cpp_classifier, concave, points) == [0,1,2,2,1]
    tilted = dict(concave, laser_to_body_xyz_m=[.1,-.2,.03], laser_to_body_rpy_rad=[.3,-.5,.7])
    points = [(x,y,z) for x in (-2.,-.5,0.,.5,1.,2.) for y in (-1.,0.,1.) for z in (-.8,0.,.8)]
    classes = assert_parity(cpp_classifier, tilted, points)
    assert set(classes) == {0,1,2}


def test_scan_and_cloud_keep_external_and_unknown_returns():
    config = dict(hardware_crop(), laser_to_body_xyz_m=[0.,0.,0.],
                  body_polygon_xy_m=[-.1,-.1,.1,-.1,.1,.1,-.1,.1],
                  body_z_bounds_m=[-.1,.1], boundary_ambiguity_m=.01)
    values = [.05, .100001, math.nan, math.inf, .015, .25]
    ranges, intensities, points = crop.filter_returns(values, 0., 0., .02, 8., crop.ChassisCrop(config))
    assert math.isnan(ranges[0]) and math.isnan(ranges[2])
    assert ranges[1] == .100001 and ranges[3] == math.inf and ranges[4] == math.inf and ranges[5] == .25
    assert len(points) == 2 and points[0][0] > .1 and points[1][0] == .25
    assert math.isnan(intensities[0]) and math.isnan(intensities[2])
    pose = config['laser_to_body_xyz_m']+config['laser_to_body_rpy_rad']
    crop.validate_laser_pose(config, pose)
    pose[2] += .001
    with pytest.raises(ValueError, match='differs'):
        crop.validate_laser_pose(config, pose)


def test_real_rclpy_adapter_array_overrides_and_scan_cloud_contract(monkeypatch):
    """Exercise Humble parameter typing and callbacks without running drive."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan
    monkeypatch.syspath_prepend(str(SIM))
    from duojin02_sim.adapter import DeviceAdapter

    config = dict(hardware_crop(), laser_to_body_xyz_m=[0., 0., 0.],
                  body_polygon_xy_m=[-.1,-.1,.1,-.1,.1,.1,-.1,.1],
                  body_z_bounds_m=[-.1,.1], boundary_ambiguity_m=.01)
    parameters = [Parameter('self_filter.'+key, value=value) for key, value in config.items()]
    parameters += [Parameter('published_range_min_m', value=.02),
                   Parameter('published_range_max_m', value=8.)]

    class BufferPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    context, node, disabled = Context(), None, None
    rclpy.init(args=[], context=context)
    try:
        node = DeviceAdapter(context=context, parameter_overrides=parameters)
        for timer in node.timers:
            timer.cancel()
        for key in ('laser_to_body_xyz_m', 'laser_to_body_rpy_rad', 'body_polygon_xy_m', 'body_z_bounds_m'):
            assert node.get_parameter('self_filter.'+key).type_ == Parameter.Type.DOUBLE_ARRAY
        assert node.crop.enabled
        node.scan_pub, node.cloud_pub = BufferPublisher(), BufferPublisher()
        message = LaserScan()
        message.header.stamp.sec = 1
        message.angle_min, message.angle_increment, message.angle_max = 0., 0., 0.
        message.ranges = [.05, .100001, math.nan, math.inf, .015, .25]
        node.scan_cb(message)
        scan, cloud = node.scan_pub.messages[0], node.cloud_pub.messages[0]
        assert math.isnan(scan.ranges[0]) and math.isnan(scan.ranges[2])
        assert scan.ranges[1] > .1 and scan.ranges[3] == math.inf
        assert scan.ranges[4] == math.inf and scan.ranges[5] == .25
        assert cloud.header == scan.header and cloud.width == 2
        points = list(struct.iter_unpack('<ffff', bytes(cloud.data)))
        expected = crop.filter_returns([.05, scan.ranges[1], math.nan, math.inf, .015, .25],
                                       0., 0., .02, 8., node.crop)[2]
        assert points == expected
        assert all(not node.crop.should_remove(point[:3]) for point in points)
        node.destroy_node()
        node = None

        # Typed-but-unset vectors stay optional when cropping is disabled.
        disabled = DeviceAdapter(context=context)
        for timer in disabled.timers:
            timer.cancel()
        assert not disabled.crop.enabled
    finally:
        if node is not None:
            node.destroy_node()
        if disabled is not None:
            disabled.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('polygon', [[0.,0.,1.,0.,1.,0.,0.,1.], [0.,0.,1.,1.,0.,1.,1.,0.],
                                     [0.,0.,1.,0.,.5,0.,0.,1.]])
def test_invalid_polygon_is_refused(polygon):
    with pytest.raises(ValueError):
        crop.ChassisCrop(dict(hardware_crop(), body_polygon_xy_m=polygon))


def test_saved_real_echoes_match_cpp_and_preserve_model_external(cpp_classifier):
    requested = os.environ.get('DUOJIN_REAL_LIDAR_ECHO_CSV')
    path = Path(requested) if requested else (Path.home()/'Documents/Codex/2026-10-13/outputs/real-lidar-cables-20261013/analysis/echoes.csv')
    if not path.is_file():
        if requested:
            pytest.fail('DUOJIN_REAL_LIDAR_ECHO_CSV does not exist: '+str(path))
        pytest.skip('optional saved physical capture unavailable')
    points, external = [], []
    with path.open() as stream:
        for row in csv.DictReader(stream):
            if int(row['raw_distance_mm']) == 0:
                continue
            points.append((float(row['laser_x_m']), float(row['laser_y_m']), 0.))
            external.append(row['footprint_label'] == 'outside')
    assert len(points) == 87372
    classes = assert_parity(cpp_classifier, hardware_crop(), points)
    assert all(label != crop.ChassisCrop.INSIDE for label, outside in zip(classes, external) if outside)
    assert classes.count(crop.ChassisCrop.INSIDE) > 0
