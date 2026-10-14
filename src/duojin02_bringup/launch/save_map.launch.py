from launch import LaunchDescription
from duojin02_bringup.middleware import middleware
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from duojin02_slam_tools.save_map import DEFAULT_SAVE_TIMEOUT


def saved_map_exited(event, context):
    if event.returncode != 0:
        raise RuntimeError(f"Map save client exited with code {event.returncode}")
    return []


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    output_dir = LaunchConfiguration("output_dir")

    save_map_client = Node(
        package="duojin02_slam_tools",
        executable="save_map_client_node",
        name="save_map_client_node",
        output="screen",
        parameters=[
            {
                "map_name": map_name,
                "output_dir": output_dir,
            },
            {
                "save_timeout": ParameterValue(LaunchConfiguration("save_timeout"), value_type=float),
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
            DeclareLaunchArgument("save_timeout", default_value=str(DEFAULT_SAVE_TIMEOUT)),
            DeclareLaunchArgument("serialize_graph", default_value="true"),
            RegisterEventHandler(OnProcessExit(target_action=save_map_client, on_exit=saved_map_exited)),
            save_map_client,
        ]
    )
