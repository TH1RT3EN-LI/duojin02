"""The sole chassis configuration authority and ROS-independent parameter renderer.

Only chassis.yaml is authored. Standard ROS parameter files and simulator input
are disposable, deterministic derivatives, never independently maintained.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import tempfile

import yaml

PROFILE_RELATIVE_PATH = 'src/duojin01_bringup/config/chassis.yaml'
SCHEMA_VERSION = 1
ENVIRONMENT_KEYS = frozenset({
    'OMP_NUM_THREADS', 'DUOJIN_GPU_COST_KERNEL',
    'DUOJIN_GPU_ROLLOUT_THREADS', 'DUOJIN_GPU_COST_THREADS',
})
COMPONENTS = frozenset({
    'navigation', 'guard', 'base', 'ekf', 'health', 'slam', 'twist_mux',
    'teleop_normal', 'teleop_slow', 'foxglove', 'joy', 'joy_launcher',
    'joy_axis_selector', 'recording_qos', 'keyboard',
})


class ProfileError(ValueError):
    """An invalid or ambiguous configuration must fail before device startup."""


class UniqueKeysLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ProfileError(f'duplicate YAML key: {key!r}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeysLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def default_profile() -> str:
    selected = os.environ.get('DUOJIN_CHASSIS_PROFILE')
    if selected:
        return str(Path(selected).expanduser().resolve(strict=True))
    # Source tools do not require ROS. Installed launch files resolve one
    # installed master, never fall back to an older collection of YAML files.
    source = Path(__file__).resolve().parents[1] / 'config/chassis.yaml'
    if source.is_file():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('duojin01_bringup')) / 'config/chassis.yaml')


def _mapping(value, label):
    if not isinstance(value, dict) or not value:
        raise ProfileError(f'{label} must be a nonempty mapping')
    return value


def _keys(value, expected, label):
    _mapping(value, label)
    if set(value) != set(expected):
        raise ProfileError(f'{label} keys differ: missing={sorted(set(expected)-set(value))}, '
                           f'unknown={sorted(set(value)-set(expected))}')


def _number(value, label, *, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProfileError(f'{label} must be a finite number')
    if positive and value <= 0 or nonnegative and value < 0:
        raise ProfileError(f'{label} must be {"positive" if positive else "nonnegative"}')


def _vector(value, size, label):
    if not isinstance(value, list) or len(value) != size:
        raise ProfileError(f'{label} must contain {size} numbers')
    for item in value:
        _number(item, label)


def _finite_tree(value, label='profile'):
    if isinstance(value, float) and not math.isfinite(value):
        raise ProfileError(f'{label} contains a nonfinite number')
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ProfileError(f'{label} keys must be strings')
            _finite_tree(item, label + '.' + key)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _finite_tree(item, f'{label}[{index}]')


def validate(profile):
    _keys(profile, {'schema_version', 'motion', 'geometry', 'lidar',
                    'runtime_environment', 'offline_mapping', 'components', 'simulation'}, 'profile')
    if type(profile['schema_version']) is not int or profile['schema_version'] != SCHEMA_VERSION:
        raise ProfileError('unsupported chassis configuration schema')
    _finite_tree(profile)
    motion = profile['motion']
    _keys(motion, {'forward_mps', 'reverse_mps', 'lateral_mps', 'yaw_radps', 'narrow_mps'}, 'motion')
    for key, value in motion.items():
        _number(value, 'motion.' + key, positive=True)
    geometry = profile['geometry']
    _keys(geometry, {'body_polygon_xy_m', 'body_z_bounds_m'}, 'geometry')
    polygon = geometry['body_polygon_xy_m']
    if not isinstance(polygon, list) or len(polygon) < 6 or len(polygon) % 2:
        raise ProfileError('body polygon must have at least three XY pairs')
    for value in polygon:
        _number(value, 'body_polygon_xy_m')
    vertices = list(zip(polygon[::2], polygon[1::2]))
    area = sum(x * vertices[(i+1) % len(vertices)][1] - y * vertices[(i+1) % len(vertices)][0]
               for i, (x, y) in enumerate(vertices))
    if abs(area) < 1e-12 or len(set(vertices)) != len(vertices):
        raise ProfileError('body polygon is degenerate')
    _vector(geometry['body_z_bounds_m'], 2, 'body_z_bounds_m')
    if geometry['body_z_bounds_m'][0] >= geometry['body_z_bounds_m'][1]:
        raise ProfileError('body Z bounds must be increasing')
    lidar = profile['lidar']
    _keys(lidar, {'min_range_m', 'max_range_m', 'laser_to_body_xyz_m', 'laser_to_body_rpy_rad',
                  'crop_enabled', 'boundary_ambiguity_m', 'publish_raw', 'raw_scan_topic',
                  'raw_pointcloud_topic', 'driver_parameters'}, 'lidar')
    _number(lidar['min_range_m'], 'lidar.min_range_m', nonnegative=True)
    _number(lidar['max_range_m'], 'lidar.max_range_m', positive=True)
    if lidar['min_range_m'] >= lidar['max_range_m']:
        raise ProfileError('lidar range bounds must be increasing')
    _vector(lidar['laser_to_body_xyz_m'], 3, 'laser_to_body_xyz_m')
    _vector(lidar['laser_to_body_rpy_rad'], 3, 'laser_to_body_rpy_rad')
    _number(lidar['boundary_ambiguity_m'], 'boundary_ambiguity_m', nonnegative=True)
    for key in ('crop_enabled', 'publish_raw'):
        if type(lidar[key]) is not bool:
            raise ProfileError(f'lidar.{key} must be boolean')
    driver = _mapping(lidar['driver_parameters'], 'lidar.driver_parameters')
    if any(key.startswith('self_filter.') or key in {'min_range', 'max_range'} for key in driver):
        raise ProfileError('lidar.driver_parameters contains derived fields')
    for key in ('raw_scan_topic', 'raw_pointcloud_topic'):
        if not isinstance(lidar[key], str) or not lidar[key].startswith('/'):
            raise ProfileError(f'lidar.{key} must be an absolute ROS topic')
    if lidar['publish_raw'] and (lidar['raw_scan_topic'] == driver.get('laserscan_topic') or
                                lidar['raw_pointcloud_topic'] == driver.get('pointcloud_topic')):
        raise ProfileError('raw and filtered lidar topics must differ')
    env = profile['runtime_environment']
    _keys(env, ENVIRONMENT_KEYS, 'runtime_environment')
    if env['DUOJIN_GPU_COST_KERNEL'] not in {'interval', 'scalar'}:
        raise ProfileError('unknown GPU cost kernel')
    for key in ENVIRONMENT_KEYS - {'DUOJIN_GPU_COST_KERNEL'}:
        if not isinstance(env[key], str) or not env[key].isdigit() or int(env[key]) <= 0:
            raise ProfileError(f'{key} must be a positive integer string')
    components = profile['components']
    _keys(components, COMPONENTS, 'components')
    for key, value in components.items():
        _mapping(value, 'components.' + key)
    try:
        nav = components['navigation']
        follow = nav['controller_server']['ros__parameters']['FollowPath']
        smoother = nav['velocity_smoother']['ros__parameters']
        forbidden = {'vx_max', 'vx_min', 'vy_max', 'wz_max', 'narrow_speed_max'} & set(follow)
        forbidden |= {'max_velocity', 'min_velocity'} & set(smoother)
        if forbidden:
            raise ProfileError('navigation duplicates derived motion fields: ' + ', '.join(sorted(forbidden)))
        if 'laser_min_range' in nav['amcl']['ros__parameters']:
            raise ProfileError('AMCL duplicates the derived lidar floor')
        for key in ('local_costmap', 'global_costmap'):
            if 'footprint' in nav[key][key]['ros__parameters']:
                raise ProfileError('navigation duplicates the derived chassis footprint')
        if 'footprint' in components['guard']['velocity_guard']['ros__parameters']:
            raise ProfileError('guard duplicates the derived chassis footprint')
        if 'min_laser_range' in components['slam']['slam_toolbox']['ros__parameters']:
            raise ProfileError('SLAM duplicates the derived lidar floor')
        simulation = profile['simulation']
        sim_lidar = simulation['lidar']
        if {'self_filter', 'self_filter_source_sha256', 'range_min_m', 'published_range_max_m'} & set(sim_lidar):
            raise ProfileError('simulation duplicates derived hardware lidar fields')
        _number(sim_lidar['scan_height_m'], 'simulation.lidar.scan_height_m')
        if simulation['base']['mode'] != 'kinematic':
            raise ProfileError('only the validated kinematic simulator mode is supported')
    except (KeyError, TypeError) as error:
        raise ProfileError(f'missing component structure: {error}') from error
    _mapping(profile['offline_mapping'], 'offline_mapping')
    return profile


def load_profile(path=None):
    selected = Path(path or default_profile()).expanduser().resolve(strict=True)
    try:
        return validate(yaml.load(selected.read_text(), Loader=UniqueKeysLoader))
    except yaml.YAMLError as error:
        raise ProfileError(f'invalid YAML in {selected}: {error}') from error


def _crop(profile):
    lidar = profile['lidar']
    return dict(enabled=lidar['crop_enabled'], laser_to_body_xyz_m=copy.deepcopy(lidar['laser_to_body_xyz_m']),
                laser_to_body_rpy_rad=copy.deepcopy(lidar['laser_to_body_rpy_rad']),
                body_polygon_xy_m=copy.deepcopy(profile['geometry']['body_polygon_xy_m']),
                body_z_bounds_m=copy.deepcopy(profile['geometry']['body_z_bounds_m']),
                boundary_ambiguity_m=lidar['boundary_ambiguity_m'])


def render(profile, component):
    validate(profile)
    if component == 'lidar':
        values = copy.deepcopy(profile['lidar']['driver_parameters'])
        values.update(min_range=profile['lidar']['min_range_m'], max_range=profile['lidar']['max_range_m'])
        values.update({'self_filter.' + key: value for key, value in _crop(profile).items()})
        values.update({'self_filter.' + key: profile['lidar'][key]
                       for key in ('publish_raw', 'raw_scan_topic', 'raw_pointcloud_topic')})
        return {'lslidar_driver_node': {'ros__parameters': values}}
    if component == 'slam_offline':
        result = render(profile, 'slam')
        result['slam_toolbox']['ros__parameters'].update(copy.deepcopy(profile['offline_mapping']))
        return result
    if component not in COMPONENTS:
        raise ProfileError('unknown chassis component: ' + component)
    result = copy.deepcopy(profile['components'][component])
    if component == 'navigation':
        motion = profile['motion']
        result['controller_server']['ros__parameters']['FollowPath'].update(
            vx_max=motion['forward_mps'], vx_min=-motion['reverse_mps'], vy_max=motion['lateral_mps'],
            wz_max=motion['yaw_radps'], narrow_speed_max=motion['narrow_mps'])
        result['velocity_smoother']['ros__parameters'].update(
            max_velocity=[motion['forward_mps'], motion['lateral_mps'], motion['yaw_radps']],
            min_velocity=[-motion['reverse_mps'], -motion['lateral_mps'], -motion['yaw_radps']])
        result['amcl']['ros__parameters']['laser_min_range'] = profile['lidar']['min_range_m']
        vertices = list(zip(profile['geometry']['body_polygon_xy_m'][::2],
                            profile['geometry']['body_polygon_xy_m'][1::2]))
        for key in ('local_costmap', 'global_costmap'):
            result[key][key]['ros__parameters']['footprint'] = json.dumps(vertices)
    elif component == 'guard':
        result['velocity_guard']['ros__parameters']['footprint'] = copy.deepcopy(profile['geometry']['body_polygon_xy_m'])
    elif component == 'slam':
        result['slam_toolbox']['ros__parameters']['min_laser_range'] = profile['lidar']['min_range_m']
    return result


def simulation_profile(profile_path=None):
    profile = load_profile(profile_path)
    result = copy.deepcopy(profile['simulation'])
    result['lidar'].update(range_min_m=profile['lidar']['min_range_m'],
                           published_range_max_m=profile['lidar']['max_range_m'], self_filter=_crop(profile))
    return result


def set_laser_pose(urdf_root, xyz, rpy):
    """Use the configured optical frame without moving CAD mesh geometry."""
    import xml.etree.ElementTree as ET
    _vector(xyz, 3, 'laser XYZ')
    _vector(rpy, 3, 'laser RPY')
    if urdf_root.find("link[@name='base_link']") is None:
        raise ProfileError('robot description lacks base_link')
    joints = [joint for joint in urdf_root.findall('joint')
              if joint.find('child') is not None and joint.find('child').get('link') == 'laser']
    if len(joints) != 1 or joints[0].get('type') != 'fixed':
        raise ProfileError('laser must have exactly one fixed parent joint')
    joint = joints[0]
    joint.find('parent').set('link', 'base_link')
    origin = joint.find('origin')
    if origin is None:
        origin = ET.SubElement(joint, 'origin')
    origin.set('xyz', ' '.join(format(value, '.17g') for value in xyz))
    origin.set('rpy', ' '.join(format(value, '.17g') for value in rpy))
    return urdf_root


def robot_description(profile_path, description_xacro):
    import xacro
    import xml.etree.ElementTree as ET
    profile = load_profile(profile_path)
    tree = ET.fromstring(xacro.process_file(str(description_xacro)).toxml())
    set_laser_pose(tree, profile['lidar']['laser_to_body_xyz_m'], profile['lidar']['laser_to_body_rpy_rad'])
    return ET.tostring(tree, encoding='unicode')


def payload(profile, component):
    return yaml.safe_dump(render(profile, component), sort_keys=False, allow_unicode=True).encode()


def component_file(component, profile_path=None):
    data = payload(load_profile(profile_path), component)
    directory = Path(tempfile.gettempdir()) / f'duojin-chassis-{os.getuid()}'
    directory.mkdir(mode=0o700, exist_ok=True)
    info = directory.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
        raise ProfileError('unsafe chassis runtime directory')
    destination = directory / (component + '-' + hashlib.sha256(data).hexdigest() + '.yaml')
    if destination.is_symlink():
        raise ProfileError('runtime parameter file is a symlink')
    if not destination.exists() or destination.read_bytes() != data:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, destination)
    return str(destination)
