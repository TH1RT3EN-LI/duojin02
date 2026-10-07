"""Online mode preserves the safety envelope and typed launch parameters."""
import copy
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from duojin01_bringup.online_profile import OnlineParameters, online_parameters

CONFIG = Path(__file__).resolve().parents[1] / 'src/duojin01_bringup/config/nav2.yaml'


def test_online_local_map_uses_lidar_and_memory_without_changing_fixed_config():
    fixed = yaml.safe_load(CONFIG.read_text())
    original = copy.deepcopy(fixed)
    local = fixed['local_costmap']['local_costmap']['ros__parameters']
    local['plugins'].append('custom')
    local['custom'] = {'plugin': 'example::CustomLayer'}
    converted = online_parameters(fixed)
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
    substitution = OnlineParameters(LaunchConfiguration('params_file'))
    rewritten = RewrittenYaml(source_file=substitution, param_rewrites={'use_sim_time': 'true'}, convert_types=True)
    path = rewritten.perform(context)
    data = yaml.safe_load(Path(path).read_text())
    local = data['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert local['obstacle_layer']['observation_sources'] == 'scan remembered_scan'
    assert local['use_sim_time'] is True
    assert substitution.perform(context) == substitution.perform(context)
