import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from duojin01_bringup.middleware import middleware
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('duojin01_bringup'), 'config', 'velocity_guard.yaml')
    return LaunchDescription([
        middleware(),
        Node(package='duojin01_navigation', executable='velocity_guard', name='velocity_guard',
             parameters=[config, {'use_sim_time': False}], output='screen'),
    ])
