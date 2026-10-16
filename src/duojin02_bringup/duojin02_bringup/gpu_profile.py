from pathlib import Path

import yaml
from launch import Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from nav2_common.launch import RewrittenYaml


GPU_PARAMETER = "controller_server.ros__parameters.FollowPath.use_gpu"
OPTIMIZER_PARAMETER = "controller_server.ros__parameters.FollowPath.optimizer_backend"
OBSERVATION_PARAMETER = "amcl.ros__parameters.observation_backend"


def parse_use_gpu(value):

    if type(value) is bool:
        return value
    if isinstance(value, str) and value.strip().lower() in ("true", "false"):
        return value.strip().lower() == "true"
    raise ValueError("use_gpu must be true or false")


def configured_use_gpu(source_file):

    data = yaml.safe_load(Path(source_file).read_text())
    follow = data["controller_server"]["ros__parameters"]["FollowPath"]
    if type(follow.get("use_gpu")) is not bool:
        raise ValueError("FollowPath.use_gpu must be a YAML boolean")
    return follow["use_gpu"]


class GpuDefault(Substitution):
    def __init__(self, source_file):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)

    def perform(self, context):
        source = perform_substitutions(context, self.source_file)
        return str(configured_use_gpu(source)).lower()

    def describe(self):
        return "CUDA preference from the selected navigation parameters"


class GpuChoice(Substitution):
    def __init__(self, value):
        super().__init__()
        self.value = normalize_to_list_of_substitutions(value)

    def perform(self, context):
        return str(parse_use_gpu(perform_substitutions(context, self.value))).lower()

    def describe(self):
        return "Validated CUDA preference"


class BackendChoice(Substitution):
    """A global CPU override; otherwise preserve each component's selected policy."""

    def __init__(self, source_file, use_gpu, parameter):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)
        self.use_gpu = normalize_to_list_of_substitutions(use_gpu)
        self.parameter = parameter

    def perform(self, context):
        if not parse_use_gpu(perform_substitutions(context, self.use_gpu)):
            return "cpu"
        data = yaml.safe_load(Path(perform_substitutions(context, self.source_file)).read_text())
        value = data
        for key in self.parameter.split("."):
            if not isinstance(value, dict) or key not in value:
                return "auto"
            value = value[key]
        if value not in ("cpu", "auto", "cuda"):
            raise ValueError(f"{self.parameter} must be cpu, auto or cuda")
        return value

    def describe(self):
        return f"CUDA backend policy for {self.parameter}"


def use_gpu_argument():
    return DeclareLaunchArgument(
        "use_gpu",
        default_value=GpuDefault(LaunchConfiguration("params_file")),
        choices=["true", "false"],
        description="Enable configured CUDA geometry, MPPI and AMCL policies; false forces CPU",
    )


def gpu_parameters(source_file, use_gpu):
    return RewrittenYaml(
        source_file=source_file,
        param_rewrites={
            GPU_PARAMETER: GpuChoice(use_gpu),
            OPTIMIZER_PARAMETER: BackendChoice(source_file, use_gpu, OPTIMIZER_PARAMETER),
            OBSERVATION_PARAMETER: BackendChoice(source_file, use_gpu, OBSERVATION_PARAMETER),
        },
        convert_types=True,
    )
