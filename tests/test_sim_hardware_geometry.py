"""Hardware-authority laser alignment; synthetic poses are not physical calibration."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
from conftest import CHASSIS_PROFILE, simulation_package
from duojin01_bringup.chassis_profile import load_profile, simulation_profile

MODULE = simulation_package()/'duojin02_sim/hardware_geometry.py'
SPEC = importlib.util.spec_from_file_location('sim_hardware_geometry_under_test', MODULE)
geometry = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = geometry
SPEC.loader.exec_module(geometry)


def assert_transform(actual, expected, tolerance=1e-12):
    assert actual.translation == pytest.approx(expected.translation, abs=tolerance)
    assert actual.rotation == pytest.approx(expected.rotation, abs=tolerance)


def simple_urdf(parent_origin='0.0924721442817 -8.97794961929e-06 0.130232665815 0 0 0',
                laser_origin='0 0 0.013075000005 0 0 0'):
    pose = list(map(float, parent_origin.split()))
    laser = list(map(float, laser_origin.split()))
    return ET.fromstring(f'''<robot name="synthetic_sim">
      <link name="base_link"/><link name="lidar_link"><visual name="body">
        <origin xyz="0.01 0.02 0.03" rpy="0.1 0.2 0.3"/>
        <geometry><box size="0.07 0.05 0.036"/></geometry></visual></link><link name="laser"/>
      <joint name="lidar_mount" type="fixed"><parent link="base_link"/><child link="lidar_link"/>
        <origin xyz="{' '.join(map(str, pose[:3]))}" rpy="{' '.join(map(str, pose[3:]))}"/></joint>
      <joint name="laser_frame" type="fixed"><parent link="lidar_link"/><child link="laser"/>
        <origin xyz="{' '.join(map(str, laser[:3]))}" rpy="{' '.join(map(str, laser[3:]))}"/></joint>
    </robot>''')


def simple_model(base_pose='0 0 0 0 0 0', lidar_pose=None, relative_to=None):
    lidar_pose = lidar_pose or '0.0924721442817 -8.97794961929e-06 0.130232665815 0 0 0'
    relative = f' relative_to="{relative_to}"' if relative_to is not None else ''
    return ET.fromstring(f'''<model name="synthetic_model"><pose>4 5 6 0 0 0.8</pose>
      <link name="base_link"><pose>{base_pose}</pose></link>
      <link name="lidar_link"><pose{relative}>{lidar_pose}</pose>
        <inertial><mass>0.06</mass><pose>0.01 0.02 0.03 0 0 0</pose></inertial>
        <visual name="body"><pose>0.01 0.02 0.03 0.1 0.2 0.3</pose>
          <geometry><box><size>0.07 0.05 0.036</size></box></geometry></visual>
        <collision name="body"><pose>0.01 0.02 0.03 0.1 0.2 0.3</pose></collision>
        <sensor name="lidar" type="gpu_lidar"><pose>0 0 0.013075000005 0 0 0</pose></sensor>
      </link><frame name="laser" attached_to="lidar_link">
        <pose relative_to="lidar_link">0 0 0.013075000005 0 0 0</pose></frame>
    </model>''')


def body_snapshot(model):
    link = copy.deepcopy(model.find("link[@name='lidar_link']"))
    for sensor in list(link.findall('sensor')):
        link.remove(sensor)
    return ET.tostring(link)


def synthetic_hardware(tmp_path, first='1 2 3 0 0 1.5707963267948966',
                       second='0.5 -0.25 0.4 0.2 -0.3 0.4'):
    pytest.importorskip('xacro')
    urdf = simple_urdf(first, second)
    path = tmp_path / 'authority.xacro'
    path.write_text(ET.tostring(urdf, encoding='unicode'))
    return path


def test_full_pose_composition_inverse_and_gimbal_roundtrip():
    left = geometry.pose_transform('1 2 3 0 0 1.5707963267948966')
    right = geometry.pose_transform('0.5 -0.25 0.4 0.2 -0.3 0.4')
    expected = geometry.pose_transform([1.25, 2.5, 3.4, 0.2, -0.3, math.pi/2+0.4])
    actual = geometry.compose(left, right)
    assert_transform(actual, expected)
    assert_transform(geometry.compose(actual, geometry.inverse(actual)), geometry.SE3())
    for pitch in (0.31, math.pi/2, -math.pi/2):
        transform = geometry.pose_transform([0.2, -0.1, 0.8, 0.2, pitch, -0.7])
        assert_transform(geometry.pose_transform(geometry.transform_pose(transform)), transform)


def test_fixed_transform_composes_both_sides_of_common_ancestor():
    urdf = simple_urdf('1 2 3 0 0 1.5707963267948966', '0.5 -0.25 0.4 0.2 -0.3 0.4')
    expected = geometry.pose_transform([1.25, 2.5, 3.4, 0.2, -0.3, math.pi/2+0.4])
    assert_transform(geometry.fixed_transform(urdf, 'base_link', 'laser'), expected)
    assert_transform(geometry.fixed_transform(urdf, 'laser', 'base_link'), geometry.inverse(expected))
    ET.SubElement(urdf, 'link', name='other')
    joint = ET.SubElement(urdf, 'joint', name='other_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='other')
    ET.SubElement(joint, 'origin', xyz='0.2 0.3 0.4', rpy='0.1 0.2 0.3')
    assert_transform(geometry.fixed_transform(urdf, 'other', 'laser'),
                     geometry.compose(geometry.inverse(geometry.pose_transform('0.2 0.3 0.4 0.1 0.2 0.3')), expected))


@pytest.mark.parametrize('damage', ['unknown', 'cycle', 'moving', 'duplicate_parent', 'disconnected'])
def test_invalid_fixed_chains_are_refused(damage):
    urdf = simple_urdf()
    if damage == 'unknown':
        urdf.find("joint[@name='laser_frame']/parent").set('link', 'unknown')
    elif damage == 'cycle':
        urdf.find("joint[@name='lidar_mount']/parent").set('link', 'laser')
    elif damage == 'moving':
        urdf.find("joint[@name='lidar_mount']").set('type', 'revolute')
    elif damage == 'duplicate_parent':
        urdf.append(copy.deepcopy(urdf.find("joint[@name='laser_frame']")))
    else:
        urdf.remove(urdf.find("joint[@name='lidar_mount']"))
    with pytest.raises(ValueError):
        geometry.fixed_transform(urdf, 'base_link', 'laser')


@pytest.mark.parametrize('pose', ['0 0 0 0 0', '0 0 0 0 0 nan', '0 0 inf 0 0 0'])
def test_invalid_pose_values_are_refused(pose):
    with pytest.raises(ValueError):
        geometry.pose_transform(pose)


def test_real_hardware_chain_clears_old_13mm_scan_offset_and_preserves_body(tmp_path):
    pytest.importorskip('xacro')
    description = ROOT / 'src/duojin01_description'
    hardware = geometry.expand_hardware_urdf(description)
    hardware_pose = geometry.transform_pose(geometry.fixed_transform(hardware, 'base_link', 'laser'))
    assert hardware_pose == pytest.approx([0.0924721442817, -8.97794961929e-06,
                                          0.1302326658155, 0, 0, 0], abs=1e-13)
    model, urdf = simple_model(), simple_urdf()
    before_body = body_snapshot(model)
    before_urdf_body = ET.tostring(urdf.find("link[@name='lidar_link']"))
    result = geometry.align_laser(model, urdf, description, tmp_path)
    assert result['hardware_laser_xyz_rpy'] == pytest.approx(hardware_pose, abs=1e-13)
    assert result['simulation_laser_xyz_rpy'] == pytest.approx(hardware_pose, abs=1e-13)
    assert result['sensor_relative_xyz_rpy'] == pytest.approx([0, 0, 5e-13, 0, 0, 0], abs=1e-13)
    assert result['sim_laser_joint_relative_xyz_rpy'] == pytest.approx([0, 0, 5e-13, 0, 0, 0], abs=1e-13)
    assert body_snapshot(model) == before_body
    assert ET.tostring(urdf.find("link[@name='lidar_link']")) == before_urdf_body
    assert_transform(geometry.fixed_transform(urdf, 'base_link', 'laser'),
                     geometry.pose_transform(hardware_pose))
    sensor_pose = model.find("link[@name='lidar_link']/sensor/pose")
    frame_pose = model.find("frame[@name='laser']/pose")
    assert sensor_pose.text == frame_pose.text
    assert sensor_pose.get('relative_to') == frame_pose.get('relative_to') == 'lidar_link'
    assert len(result['source_xacro_hashes']) == 3
    assert {Path(row['path']).name for row in result['source_xacro_hashes']} == {
        'duojin01.xacro', 'duojin01_hardware.xacro', 'chassis_geometry.xacro'}
    for row in result['source_xacro_hashes']:
        assert row['sha256'] == hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()
    assert result['expanded_hardware_urdf_sha256'] == hashlib.sha256(ET.tostring(hardware, encoding='utf-8')).hexdigest()
    assert result['optical_height_calibrated'] is False
    assert json.loads((tmp_path / 'hardware-laser.json').read_text()) == result


def test_nonzero_hardware_authority_updates_full_xyz_rpy_in_both_sim_chains(tmp_path):
    hardware = synthetic_hardware(tmp_path)
    model = simple_model('2 -1 0.5 0.1 0.2 -0.4', '0.12 -0.06 0.25 0.4 -0.2 0.3', 'base_link')
    # The RSP parent chain deliberately differs from the SDF sensor body chain.
    urdf = simple_urdf('0.2 -0.3 0.4 -0.2 0.3 -0.4', '0 0 0.013075 0.5 0.2 0.1')
    body_before = body_snapshot(model)
    result = geometry.align_laser(model, urdf, hardware, tmp_path / 'result')
    expected = geometry.pose_transform([1.25, 2.5, 3.4, 0.2, -0.3, math.pi/2+0.4])
    assert_transform(geometry.pose_transform(result['hardware_laser_xyz_rpy']), expected)
    assert_transform(geometry.compose(geometry.pose_transform(result['sim_base_to_lidar_xyz_rpy']),
                                      geometry.pose_transform(result['sensor_relative_xyz_rpy'])), expected)
    assert_transform(geometry.fixed_transform(urdf, 'base_link', 'laser'), expected)
    assert body_snapshot(model) == body_before
    assert any(abs(v) > 0.1 for v in result['sensor_relative_xyz_rpy'][3:])
    assert any(abs(v) > 0.1 for v in result['sim_laser_joint_relative_xyz_rpy'][3:])


def test_explicit_sdf_base_relative_to_lidar_is_resolved(tmp_path):
    hardware = synthetic_hardware(tmp_path)
    model = simple_model('0.1 0.2 0.3 0.1 0.2 0.3', '1 2 3 0.3 -0.2 0.1')
    model.find("link[@name='base_link']/pose").set('relative_to', 'lidar_link')
    result = geometry.align_laser(model, simple_urdf(), hardware, tmp_path / 'result')
    assert_transform(geometry.pose_transform(result['sim_base_to_lidar_xyz_rpy']),
                     geometry.inverse(geometry.pose_transform('0.1 0.2 0.3 0.1 0.2 0.3')))


@pytest.mark.parametrize('damage', ['unknown', 'cycle', 'degrees', 'quaternion'])
def test_unknown_or_cyclic_sdf_pose_references_and_formats_are_refused(tmp_path, damage):
    hardware = synthetic_hardware(tmp_path)
    model = simple_model()
    pose = model.find("link[@name='lidar_link']/pose")
    if damage == 'unknown':
        pose.set('relative_to', 'some_arbitrary_frame')
    elif damage == 'cycle':
        pose.set('relative_to', 'base_link')
        model.find("link[@name='base_link']/pose").set('relative_to', 'lidar_link')
    elif damage == 'degrees':
        pose.set('degrees', 'true')
    else:
        pose.set('rotation_format', 'quat_xyzw')
    before = ET.tostring(model)
    with pytest.raises(ValueError):
        geometry.align_laser(model, simple_urdf(), hardware, tmp_path / 'result')
    assert ET.tostring(model) == before


def test_sim_laser_uses_its_own_multilink_parent_chain(tmp_path):
    hardware = synthetic_hardware(tmp_path)
    urdf = simple_urdf('0.1 0.2 0.3 0.2 0.1 -0.4')
    ET.SubElement(urdf, 'link', name='optical_mount')
    joint = ET.SubElement(urdf, 'joint', name='optical_mount_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='lidar_link')
    ET.SubElement(joint, 'child', link='optical_mount')
    ET.SubElement(joint, 'origin', xyz='0.04 -0.03 0.02', rpy='-0.3 0.2 0.1')
    urdf.find("joint[@name='laser_frame']/parent").set('link', 'optical_mount')
    result = geometry.align_laser(simple_model(), urdf, hardware, tmp_path / 'result')
    assert result['sim_laser_joint_parent'] == 'optical_mount'
    assert_transform(geometry.fixed_transform(urdf, 'base_link', 'laser'),
                     geometry.pose_transform([1.25, 2.5, 3.4, 0.2, -0.3, math.pi/2+0.4]))


def test_explicit_scan_height_overrides_only_body_frame_z_through_both_full_chains(tmp_path):
    hardware = synthetic_hardware(tmp_path)
    model = simple_model('2 -1 0.5 0.1 0.2 -0.4', '0.12 -0.06 0.25 0.4 -0.2 0.3', 'base_link')
    urdf = simple_urdf('0.1 0.2 0.3 0.2 0.1 -0.4')
    ET.SubElement(urdf, 'link', name='optical_mount')
    joint = ET.SubElement(urdf, 'joint', name='optical_mount_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='lidar_link')
    ET.SubElement(joint, 'child', link='optical_mount')
    ET.SubElement(joint, 'origin', xyz='0.04 -0.03 0.02', rpy='-0.3 0.2 0.1')
    urdf.find("joint[@name='laser_frame']/parent").set('link', 'optical_mount')
    source_before = hardware.read_bytes()
    body_before = body_snapshot(model)
    urdf_body_before = ET.tostring(urdf.find("link[@name='lidar_link']"))
    mount_before = ET.tostring(urdf.find("joint[@name='optical_mount_joint']"))
    result = geometry.align_laser(model, urdf, hardware, tmp_path/'result',
                                  scan_height_m=0.14330766582)
    hardware_pose = [1.25, 2.5, 3.4, 0.2, -0.3, math.pi/2+0.4]
    simulation_pose = [1.25, 2.5, 0.14330766582, 0.2, -0.3, math.pi/2+0.4]
    assert result['hardware_laser_xyz_rpy'] == pytest.approx(hardware_pose, abs=1e-12)
    assert result['simulation_laser_xyz_rpy'] == pytest.approx(simulation_pose, abs=1e-12)
    expected = geometry.pose_transform(simulation_pose)
    assert_transform(geometry.compose(geometry.pose_transform(result['sim_base_to_lidar_xyz_rpy']),
                                      geometry.pose_transform(result['sensor_relative_xyz_rpy'])), expected)
    assert_transform(geometry.fixed_transform(urdf, 'base_link', 'laser'), expected)
    assert_transform(geometry.compose(geometry.pose_transform(result['sim_base_to_laser_parent_xyz_rpy']),
                                      geometry.pose_transform(result['sim_laser_joint_relative_xyz_rpy'])), expected)
    assert result['sim_laser_joint_parent'] == 'optical_mount'
    assert model.findtext("link[@name='lidar_link']/sensor/pose") == model.findtext("frame[@name='laser']/pose")
    assert hardware.read_bytes() == source_before
    assert body_snapshot(model) == body_before
    assert ET.tostring(urdf.find("link[@name='lidar_link']")) == urdf_body_before
    assert ET.tostring(urdf.find("joint[@name='optical_mount_joint']")) == mount_before
    assert result['optical_height_calibrated'] is False


@pytest.mark.parametrize('height', [math.nan, math.inf, -math.inf, 'not-a-height'])
def test_nonfinite_or_invalid_scan_height_is_refused_before_mutation(tmp_path, height):
    model, urdf = simple_model(), simple_urdf()
    before_model, before_urdf = ET.tostring(model), ET.tostring(urdf)
    # Validation precedes hardware source expansion as well as XML mutation.
    with pytest.raises(ValueError):
        geometry.align_laser(model, urdf, tmp_path/'missing-authority.xacro',
                              tmp_path/'result', scan_height_m=height)
    assert ET.tostring(model) == before_model
    assert ET.tostring(urdf) == before_urdf
    assert not (tmp_path/'result/hardware-laser.json').exists()


@pytest.mark.parametrize('height', [None, 0.14330766582])
@pytest.mark.parametrize('configured_pose', [None, [0.12, -.04, .137, .10, -.12, .34]])
def test_scene_explicit_height_is_allowed_and_absent_height_keeps_hardware(tmp_path, monkeypatch, height, configured_pose):
    """Generate XML only; no Gazebo, ROS nodes or GPU are started."""
    pytest.importorskip('xacro')
    package = MODULE.parents[1]
    monkeypatch.syspath_prepend(str(package))
    scene_spec = importlib.util.spec_from_file_location('scene_height_under_test', package/'duojin02_sim/scene.py')
    scene = importlib.util.module_from_spec(scene_spec)
    scene_spec.loader.exec_module(scene)
    description = tmp_path/'description'
    (description/'models/dual_arm_mobile').mkdir(parents=True)
    (description/'urdf').mkdir()
    model = simple_model()
    for visual in model.findall('.//visual'):
        ET.SubElement(visual, 'material')
    for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        ET.SubElement(model, 'link', name=name+'_wheel')
    source_model = description/'models/dual_arm_mobile/model.sdf'
    source_urdf = description/'urdf/dual_arm_mobile.urdf'
    sdf = ET.Element('sdf', version='1.9')
    sdf.append(model)
    source_model.write_text(ET.tostring(sdf, encoding='unicode'))
    source_urdf.write_text(ET.tostring(simple_urdf(), encoding='unicode'))
    import yaml
    master = load_profile(CHASSIS_PROFILE)
    if configured_pose is not None:
        master['lidar']['laser_to_body_xyz_m'] = configured_pose[:3]
        master['lidar']['laser_to_body_rpy_rad'] = configured_pose[3:]
    selected_master = tmp_path/'chassis.yaml'
    selected_master.write_text(yaml.safe_dump(master, sort_keys=False))
    profile = simulation_profile(selected_master)
    profile['materials']['body'] = {'roughness': .4}
    profile['lidar'].pop('scan_height_m', None)
    if height is not None:
        profile['lidar']['scan_height_m'] = height
    profile_before = copy.deepcopy(profile)
    before = [path.read_bytes() for path in (source_model, source_urdf)]
    output = tmp_path/'scene'
    scene_path, aligned_urdf = scene.generate(description, profile, output, noise=False,
        observer=False, hardware_description=ROOT/'src/duojin01_description')
    metadata = json.loads((output/'hardware-laser.json').read_text())
    selected_pose = master['lidar']['laser_to_body_xyz_m'] + master['lidar']['laser_to_body_rpy_rad']
    assert metadata['hardware_laser_xyz_rpy'] == pytest.approx(selected_pose, abs=1e-13)
    assert metadata['configured_laser_xyz_rpy'] == selected_pose
    expected_z = selected_pose[2] if height is None else height
    assert metadata['simulation_laser_xyz_rpy'][2] == pytest.approx(expected_z, abs=1e-13)
    assert metadata['simulation_laser_xyz_rpy'][:2] == pytest.approx(metadata['hardware_laser_xyz_rpy'][:2])
    assert metadata['simulation_laser_xyz_rpy'][3:] == pytest.approx(metadata['hardware_laser_xyz_rpy'][3:])
    generated_model = ET.parse(scene_path).find("world/model[@name='synthetic_model']")
    sensor = generated_model.find("link[@name='lidar_link']/sensor/pose")
    assert_transform(geometry.compose(geometry.pose_transform(metadata['sim_base_to_lidar_xyz_rpy']),
                                      geometry.pose_transform(sensor.text)),
                     geometry.pose_transform(metadata['simulation_laser_xyz_rpy']))
    assert_transform(geometry.fixed_transform(ET.fromstring(aligned_urdf), 'base_link', 'laser'),
                     geometry.pose_transform(metadata['simulation_laser_xyz_rpy']))
    assert [path.read_bytes() for path in (source_model, source_urdf)] == before
    assert profile == profile_before


def test_path_inputs_write_generated_copies_without_touching_sources(tmp_path):
    hardware = synthetic_hardware(tmp_path)
    sdf, urdf = tmp_path / 'source.sdf', tmp_path / 'source.urdf'
    sdf.write_text(ET.tostring(simple_model(), encoding='unicode'))
    urdf.write_text(ET.tostring(simple_urdf(), encoding='unicode'))
    before = [path.read_bytes() for path in (sdf, urdf, hardware)]
    result = geometry.align_laser(sdf, urdf, hardware, tmp_path / 'output')
    assert [path.read_bytes() for path in (sdf, urdf, hardware)] == before
    assert Path(result['aligned_model_sdf']).is_file()
    transformed_urdf = ET.parse(result['aligned_sim_urdf']).getroot()
    assert_transform(geometry.fixed_transform(transformed_urdf, 'base_link', 'laser'),
                     geometry.pose_transform([1.25, 2.5, 3.4, 0.2, -0.3, math.pi/2+0.4]))
