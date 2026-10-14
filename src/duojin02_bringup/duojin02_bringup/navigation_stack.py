"""The active navigation graph; localization and base arbitration have owners."""
from launch_ros.actions import Node


NAVIGATION_NODES = ['controller_server', 'planner_server',
                    'bt_navigator', 'velocity_smoother']


def navigation_actions(parameters, use_sim_time, autostart, log_level, output_topic):
    actions = []
    for package, name in [('nav2_controller', 'controller_server'),
                          ('nav2_planner', 'planner_server'),
                          ('nav2_bt_navigator', 'bt_navigator'),
                          ('nav2_velocity_smoother', 'velocity_smoother')]:
        remappings = [('cmd_vel', '/cmd_vel_nav')]
        if name == 'velocity_smoother':
            remappings.append(('cmd_vel_smoothed', output_topic))
        actions.append(Node(package=package, executable=name, name=name,
            output='screen', parameters=[parameters, {'use_sim_time':use_sim_time}],
            arguments=['--ros-args','--log-level',log_level], remappings=remappings))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', parameters=[{
            'use_sim_time':use_sim_time, 'autostart':autostart, 'node_names':NAVIGATION_NODES}]))
    return actions
