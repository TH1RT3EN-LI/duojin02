from launch import LaunchDescription
from duojin01_bringup.middleware import middleware
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    output_dir = LaunchConfiguration("output_dir")
    wait_timeout = LaunchConfiguration("wait_timeout")

    save_map_client = Node(
        package="duojin01_slam_tools",
        executable="save_map_client_node",
        name="save_map_client_node",
        output="screen",
        parameters=[
            {
                "map_name": map_name,
                "output_dir": output_dir,
            },
            {
                "wait_timeout": ParameterValue(wait_timeout, value_type=float),
                "response_timeout": ParameterValue(LaunchConfiguration("response_timeout"), value_type=float),
                "serialize_graph": ParameterValue(LaunchConfiguration("serialize_graph"), value_type=bool),
                "use_sim_time": False,
            },
        ],
    )

    return LaunchDescription(
        [
            middleware(),
            DeclareLaunchArgument("map_name", default_value="auto"),
            DeclareLaunchArgument("output_dir", default_value="maps"),
            DeclareLaunchArgument("wait_timeout", default_value="30"),
            DeclareLaunchArgument("response_timeout", default_value="30"),
            DeclareLaunchArgument("serialize_graph", default_value="true"),
            save_map_client,
        ]
    )
