import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from duojin02_bringup.middleware import middleware
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin02_bringup")
    teleop_share = get_package_share_directory("duojin02_teleop")
    
    joy_launcher_config_path = os.path.join(teleop_share, "config", "joy_launcher.yaml")
    joy_axis_selector_config_path = os.path.join(teleop_share, "config", "joy_axis_selector.yaml")
    joy_teleop_normal_config_path = os.path.join(bringup_share, "config", "joy_teleop_normal.yaml")
    joy_teleop_slow_config_path = os.path.join(bringup_share, "config", "joy_teleop_slow.yaml")


    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "use_sim_time": False,
                "device_id": 0,
                "deadzone": 0.12,
                "autorepeat_rate": 20.0,
            }
        ],
        remappings=[("/joy", "/joy_raw")],  
    )
    
    joy_axis_selector = Node(
        package="duojin02_teleop",
        executable="joy_axis_selector_node",
        name="joy_axis_selector_node",
        parameters=[joy_axis_selector_config_path, {"use_sim_time": False}],
    )
    joy_teleop_slow = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_slow",
        parameters=[joy_teleop_slow_config_path, {"use_sim_time": False}],
        remappings=[("/cmd_vel", "/cmd_vel_slow")],
    )

    joy_teleop_normal = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_normal",
        parameters=[joy_teleop_normal_config_path, {"use_sim_time": False}],
        remappings=[("/cmd_vel", "/cmd_vel_normal")],
    )

    joy_launcher = Node(
        package="duojin02_teleop",
        executable="joy_launcher_node",
        name="joy_launcher_node",
        parameters=[joy_launcher_config_path, os.path.join(bringup_share, "config", "safety.yaml"),
                    {"use_sim_time": False}],
        output="screen",
    )
    return LaunchDescription(
        [
            middleware(),
            joy,
            joy_axis_selector,
            joy_teleop_slow,
            joy_teleop_normal,
            joy_launcher,
        ]
    )
