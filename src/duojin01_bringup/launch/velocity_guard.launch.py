import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('duojin01_bringup'), 'config', 'velocity_guard.yaml')
    return LaunchDescription([
        Node(package='duojin01_navigation', executable='velocity_guard', name='velocity_guard',
             parameters=[config, {'use_sim_time': False}], output='screen'),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[config, {'use_sim_time': False}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_guard',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['collision_monitor']}],
             output='screen'),
    ])
