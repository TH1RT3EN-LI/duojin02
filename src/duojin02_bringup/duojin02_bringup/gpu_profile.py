"""Select geometry acceleration without creating a second navigation profile."""
from pathlib import Path

import yaml
from launch import Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from nav2_common.launch import RewrittenYaml


GPU_PARAMETER = 'controller_server.ros__parameters.FollowPath.use_gpu'


def parse_use_gpu(value):
    """Accept only the two public launch choices and actual YAML booleans."""
    if type(value) is bool:
        return value
    if isinstance(value, str) and value.strip().lower() in ('true', 'false'):
        return value.strip().lower() == 'true'
    raise ValueError('use_gpu must be true or false')


def configured_use_gpu(source_file):
    """Read the selected profile, with the shipped profile as the sole default."""
    data = yaml.safe_load(Path(source_file).read_text())
    follow = data['controller_server']['ros__parameters']['FollowPath']
    if 'use_gpu' not in follow:
        default_file = Path(__file__).resolve().parents[1] / 'config/nav2.yaml'
        if not default_file.is_file():
            from ament_index_python.packages import get_package_share_directory
            default_file = Path(get_package_share_directory('duojin02_bringup')) / 'config/nav2.yaml'
        data = yaml.safe_load(default_file.read_text())
        follow = data['controller_server']['ros__parameters']['FollowPath']
    if type(follow['use_gpu']) is not bool:
        raise ValueError('FollowPath.use_gpu must be a YAML boolean')
    return follow['use_gpu']


class GpuDefault(Substitution):
    def __init__(self, source_file):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)

    def perform(self, context):
        source = perform_substitutions(context, self.source_file)
        return str(configured_use_gpu(source)).lower()

    def describe(self):
        return 'Geometry GPU preference from the selected navigation parameters'


class GpuChoice(Substitution):
    def __init__(self, value):
        super().__init__()
        self.value = normalize_to_list_of_substitutions(value)

    def perform(self, context):
        return str(parse_use_gpu(perform_substitutions(context, self.value))).lower()

    def describe(self):
        return 'Validated geometry GPU preference'


def use_gpu_argument():
    return DeclareLaunchArgument(
        'use_gpu', default_value=GpuDefault(LaunchConfiguration('params_file')),
        choices=['true', 'false'],
        description='Use available GPU geometry acceleration; default comes from params_file',
    )


def gpu_parameters(source_file, use_gpu):
    return RewrittenYaml(
        source_file=source_file, param_rewrites={GPU_PARAMETER: GpuChoice(use_gpu)},
        convert_types=True,
    )
