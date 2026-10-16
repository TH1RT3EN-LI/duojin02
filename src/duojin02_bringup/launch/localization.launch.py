"""Fixed-map localization using the source-pinned CUDA-capable AMCL node."""
import os

from ament_index_python.packages import get_package_share_directory
from duojin02_bringup.gpu_profile import gpu_parameters, use_gpu_argument
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params = ParameterFile(
        RewrittenYaml(
            source_file=gpu_parameters(params_file, LaunchConfiguration("use_gpu")),
            param_rewrites={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "yaml_filename": LaunchConfiguration("map"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    arguments = ["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            get_package_share_directory("duojin02_bringup"), "config", "nav2.yaml")),
        use_gpu_argument(),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[params], arguments=arguments, remappings=remappings),
        Node(package="duojin02_amcl_cuda", executable="amcl", name="amcl",
             output="screen", parameters=[params], arguments=arguments, remappings=remappings),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen", arguments=arguments,
             parameters=[{
                 "use_sim_time": use_sim_time,
                 "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
                 "node_names": ["map_server", "amcl"],
             }]),
    ])
