from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('custom_localization')

    left_params_file = os.path.join(
        pkg_dir,
        'config',
        'ublox_left.yaml'
    )

    right_params_file = os.path.join(
        pkg_dir,
        'config',
        'ublox_right.yaml'
    )

    navsat_file = os.path.join(
        pkg_dir,
        'config',
        'navsat.yaml'
    )

    ekf_file = os.path.join(
        pkg_dir,
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([

        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='left_ublox_gps',
            namespace='left',
            output='screen',
            parameters=[left_params_file]
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='right_ublox_gps',
            namespace='right',
            output='screen',
            parameters=[right_params_file]
        ),
        # Node(
        #     package='custom_localization',
        #     executable='rtk_localizer',
        #     name='rtk_localizer',
        #     output='screen'
        # )

        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',

        #     output='screen',

        #     parameters=[navsat_file],

        #     remappings=[
        #         ('gps/fix', '/fix'),
        #         ('odometry/gps', '/odometry/gps'),
        #         ('imu/data', '/imu')
        #     ]
        # ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',

        #     output='screen',

        #     parameters=[ekf_file]
        # )


    ])