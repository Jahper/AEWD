from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    custom_nav_dir = get_package_share_directory('custom_navigation')
    params_file = os.path.join(custom_nav_dir, 'config', 'nav2_params.yaml')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    # No map_server, no amcl — GPS handles localization

    return LaunchDescription([
        Node(package='nav2_controller',    executable='controller_server',   output='screen', parameters=[params_file]),
        Node(package='nav2_smoother',      executable='smoother_server',     output='screen', parameters=[params_file]),
        Node(package='nav2_planner',       executable='planner_server',      output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors',     executable='behavior_server',     output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator',  executable='bt_navigator',        output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', output='screen', parameters=[params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', output='screen', parameters=[params_file]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}]
        ),
    ])