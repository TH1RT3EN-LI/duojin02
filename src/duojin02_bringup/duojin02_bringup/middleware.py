from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable


def middleware():
    return SetEnvironmentVariable(
        "RMW_IMPLEMENTATION",
        EnvironmentVariable("RMW_IMPLEMENTATION", default_value="rmw_cyclonedds_cpp"),
    )
