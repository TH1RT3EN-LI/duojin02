"""One geometry acceleration choice reaches every navigation entry point."""
import copy
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from duojin02_bringup.gpu_profile import (
    GpuDefault, configured_use_gpu, gpu_parameters, parse_use_gpu,
)


BRINGUP = Path(__file__).resolve().parents[1] / 'src/duojin02_bringup'
CONFIG = BRINGUP / 'config/nav2.yaml'


def load_launch(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'gpu_' + name, BRINGUP / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, 'get_package_share_directory'):
        original = module.get_package_share_directory
        monkeypatch.setattr(module, 'get_package_share_directory',
                            lambda name: str(BRINGUP) if name == 'duojin02_bringup' else original(name))
    return module


def selected_profile(tmp_path, use_gpu):
    data = yaml.safe_load(CONFIG.read_text())
    data['controller_server']['ros__parameters']['FollowPath']['use_gpu'] = use_gpu
    selected = tmp_path / 'navigation.yaml'
    selected.write_text(yaml.safe_dump(data))
    return selected, data


@pytest.mark.parametrize('entry', ['navigation', 'nav2', 'navigation_stack', 'online_navigation'])
@pytest.mark.parametrize('selected', [True, False])
def test_each_entry_reads_default_from_selected_navigation_file(entry, selected, tmp_path, monkeypatch):
    path, _ = selected_profile(tmp_path, selected)
    module = load_launch(entry, monkeypatch)
    argument = next(action for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == 'use_gpu')
    context = LaunchContext()
    context.launch_configurations['params_file'] = str(path)
    argument.execute(context)
    assert context.launch_configurations['use_gpu'] == str(selected).lower()
    context.launch_configurations['use_gpu'] = str(not selected).lower()
    argument.execute(context)
    assert context.launch_configurations['use_gpu'] == str(not selected).lower()


@pytest.mark.parametrize('selected', [True, False])
@pytest.mark.parametrize('override', ['true', 'false'])
def test_final_override_changes_only_geometry_switch(selected, override, tmp_path):
    path, original = selected_profile(tmp_path, selected)
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(path), use_gpu=override)
    output = gpu_parameters(LaunchConfiguration('params_file'), LaunchConfiguration('use_gpu')).perform(context)
    actual = yaml.safe_load(Path(output).read_text())
    expected = copy.deepcopy(original)
    expected['controller_server']['ros__parameters']['FollowPath']['use_gpu'] = override == 'true'
    assert actual == expected
    assert yaml.safe_load(path.read_text()) == original


@pytest.mark.parametrize('value', ['', 'auto', 'cuda', '1', 'yes', 1, None])
def test_gpu_switch_rejects_ambiguous_choices(value):
    with pytest.raises(ValueError, match='true or false'):
        parse_use_gpu(value)


def test_yaml_switch_is_boolean_and_missing_switch_uses_shipped_default(tmp_path):
    assert configured_use_gpu(CONFIG) is True
    path, data = selected_profile(tmp_path, 'false')
    with pytest.raises(ValueError, match='YAML boolean'):
        configured_use_gpu(path)
    del data['controller_server']['ros__parameters']['FollowPath']['use_gpu']
    path.write_text(yaml.safe_dump(data))
    assert configured_use_gpu(path) == configured_use_gpu(CONFIG)
    context = LaunchContext()
    assert GpuDefault(str(path)).perform(context) == 'true'


def test_fixed_navigation_passes_choice_through_both_includes(tmp_path, monkeypatch):
    navigation = load_launch('navigation', monkeypatch)
    includes = [action for action in navigation.generate_launch_description().entities
                if isinstance(action, IncludeLaunchDescription)]
    context = LaunchContext()
    context.launch_configurations['use_gpu'] = 'false'
    arguments = dict(includes[-1].launch_arguments)
    assert perform_substitutions(context, normalize_to_list_of_substitutions(arguments['use_gpu'])) == 'false'

    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    context.launch_configurations.update(map=str(map_file), autostart='false')
    nav2 = load_launch('nav2', monkeypatch)
    actions = nav2._create_nav_actions(context, str(BRINGUP))
    nested = [child for action in actions if isinstance(action, GroupAction)
              for child in action.get_sub_entities() if isinstance(child, IncludeLaunchDescription)]
    arguments = dict(nested[-1].launch_arguments)
    assert perform_substitutions(context, normalize_to_list_of_substitutions(arguments['use_gpu'])) == 'false'


def test_online_navigation_passes_choice_to_navigation_stack(monkeypatch):
    module = load_launch('online_navigation', monkeypatch)
    captured = []

    def include(*args, **kwargs):
        result = IncludeLaunchDescription(*args, **kwargs)
        captured.append(result)
        return result

    monkeypatch.setattr(module, 'IncludeLaunchDescription', include)
    module.generate_launch_description()
    navigation = next(action for action in captured if 'use_gpu' in dict(action.launch_arguments))
    context = LaunchContext()
    context.launch_configurations['use_gpu'] = 'false'
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        dict(navigation.launch_arguments)['use_gpu'])) == 'false'


@pytest.mark.parametrize('selected,override,expected', [(False, None, False), (True, 'false', False),
                                                       (False, 'true', True)])
def test_navigation_nodes_receive_typed_effective_preference(selected, override, expected, tmp_path, monkeypatch):
    path, original = selected_profile(tmp_path, selected)
    module = load_launch('navigation_stack', monkeypatch)
    captured = []

    def navigation(parameters, *args):
        captured.append(parameters)
        return []

    monkeypatch.setattr(module, 'navigation_actions', navigation)
    context = LaunchContext()
    context.launch_configurations['params_file'] = str(path)
    if override is not None:
        context.launch_configurations['use_gpu'] = override
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    actual = yaml.safe_load(Path(captured[0].evaluate(context)).read_text())
    assert actual['controller_server']['ros__parameters']['FollowPath']['use_gpu'] is expected
    assert yaml.safe_load(path.read_text()) == original


def test_invalid_launch_choice_fails_before_navigation_start(monkeypatch):
    module = load_launch('navigation', monkeypatch)
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(CONFIG), use_gpu='auto')
    argument = next(action for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == 'use_gpu')
    with pytest.raises(RuntimeError, match='is not valid'):
        argument.execute(context)


@pytest.mark.parametrize('entry', ['base', 'mapping', 'offline_mapping'])
def test_cpu_only_workflows_do_not_advertise_gpu_acceleration(entry, monkeypatch):
    arguments = [action.name for action in load_launch(entry, monkeypatch).generate_launch_description().entities
                 if isinstance(action, DeclareLaunchArgument)]
    assert 'use_gpu' not in arguments
