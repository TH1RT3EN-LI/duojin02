"""Online mode preserves the safety envelope and typed launch parameters."""
import copy
import importlib.util
import json
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from nav2_common.launch import RewrittenYaml
from duojin01_bringup.online_profile import OnlineParameters, online_parameters

CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / 'src/duojin01_bringup/config'
CONFIG = CONFIG_DIRECTORY / 'nav2.yaml'
SLAM_CONFIG = CONFIG_DIRECTORY / 'slam_toolbox.yaml'
LATTICE = CONFIG_DIRECTORY / 'lattice_primitives/duojin01_omni_2cm_rich_lattice.json'


def test_online_local_map_uses_lidar_and_memory_without_changing_fixed_config():
    fixed = yaml.safe_load(CONFIG.read_text())
    original = copy.deepcopy(fixed)
    local = fixed['local_costmap']['local_costmap']['ros__parameters']
    local['plugins'].append('custom')
    local['custom'] = {'plugin': 'example::CustomLayer'}
    converted = online_parameters(fixed, .01, LATTICE)
    assert fixed['local_costmap']['local_costmap']['ros__parameters']['plugins'][0] == 'static_layer'
    expected = copy.deepcopy(local)
    expected['plugins'].remove('static_layer')
    assert converted['local_costmap']['local_costmap']['ros__parameters'] == expected
    assert converted['global_costmap']['global_costmap']['ros__parameters']['plugins'] == original['global_costmap']['global_costmap']['ros__parameters']['plugins']
    assert converted['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is False
    assert converted['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is True
    for name in fixed:
        if name not in ('local_costmap', 'global_costmap', 'planner_server'):
            assert converted[name] == fixed[name]


def test_online_file_resolves_launch_configuration_and_preserves_list_types():
    context = LaunchContext()
    context.launch_configurations['params_file'] = str(CONFIG)
    context.launch_configurations['slam_params_file'] = str(SLAM_CONFIG)
    substitution = OnlineParameters(LaunchConfiguration('params_file'),
                                    LaunchConfiguration('slam_params_file'), str(LATTICE))
    rewritten = RewrittenYaml(source_file=substitution, param_rewrites={'use_sim_time': 'true'}, convert_types=True)
    path = rewritten.perform(context)
    data = yaml.safe_load(Path(path).read_text())
    local = data['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert local['obstacle_layer']['observation_sources'] == 'scan remembered_scan'
    assert local['use_sim_time'] is True
    assert substitution.perform(context) == substitution.perform(context)


def test_selected_slam_config_drives_global_grid_and_lattice_at_launch_time(tmp_path):
    context = LaunchContext()
    context.launch_configurations['params_file'] = str(CONFIG)
    substitution = OnlineParameters(LaunchConfiguration('params_file'),
                                    LaunchConfiguration('slam_params_file'), str(LATTICE))
    fixed = yaml.safe_load(CONFIG.read_text())
    paths = []
    for resolution in (.03, .05):
        selected = tmp_path / f'slam-{resolution}.yaml'
        selected.write_text(yaml.safe_dump({'slam_toolbox': {
            'ros__parameters': {'resolution': resolution}}}))
        context.launch_configurations['slam_params_file'] = str(selected)
        path = substitution.perform(context)
        paths.append(path)
        data = yaml.safe_load(Path(path).read_text())
        assert data['global_costmap']['global_costmap']['ros__parameters']['resolution'] == resolution
        lattice_path = data['planner_server']['ros__parameters']['GridBased']['lattice_filepath']
        assert json.loads(Path(lattice_path).read_text())['lattice_metadata']['grid_resolution'] == resolution
        assert data['local_costmap']['local_costmap']['ros__parameters']['resolution'] == fixed['local_costmap']['local_costmap']['ros__parameters']['resolution']
    assert paths[0] != paths[1]
    assert yaml.safe_load(CONFIG.read_text()) == fixed


def test_online_launch_explicitly_forwards_the_selected_slam_configuration(monkeypatch):
    launch_file = CONFIG_DIRECTORY.parent / 'launch/online_navigation.launch.py'
    spec = importlib.util.spec_from_file_location('online_navigation_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(CONFIG_DIRECTORY.parent))
    actions = module.generate_launch_description().entities
    declared = {action.name for action in actions if isinstance(action, DeclareLaunchArgument)}
    assert {'slam_params_file', 'pose_graph', 'map_start_pose', 'start_at_dock'} <= declared
    base, slam = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert {'base_params_file', 'base_serial_baudrate', 'odom0', 'imu0'} <= dict(base.launch_arguments).keys()
    context = LaunchContext()
    for name in ('slam_params_file', 'pose_graph', 'map_start_pose', 'start_at_dock'):
        context.launch_configurations[name] = f'selected-{name}'
        value = dict(slam.launch_arguments)[name]
        assert perform_substitutions(context, normalize_to_list_of_substitutions(value)) == f'selected-{name}'
