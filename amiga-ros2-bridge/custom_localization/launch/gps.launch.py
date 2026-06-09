from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os




def generate_launch_description():

    pkg_dir = get_package_share_directory('custom_localization')


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
        DeclareLaunchArgument(
            "gps_port",
            default_value="/dev/ttyACM0"
        ),
        DeclareLaunchArgument(
            "gps_baud",
            default_value="460800"
        ),
        DeclareLaunchArgument(
            "gps_frame",
            default_value="lg580p_link"
        ),
        DeclareLaunchArgument(
            "antenna_offset_deg",
            default_value="90.0",
            description="Heading offset in degrees due to antenna placement. "
                        "90.0 = antennas side-by-side (West=0 clockwise). "
                        "0.0 = antennas front-back (North=0 clockwise, standard NMEA)."
        ),
        Node(
            package="custom_localization",
            executable="lg580p_driver_node",
            name="lg580p_driver",
            parameters=[
                {
                    "port": LaunchConfiguration("gps_port"),
                    "baud": LaunchConfiguration("gps_baud"),
                    "gps_frame": LaunchConfiguration("gps_frame"),
                    "antenna_offset_deg": LaunchConfiguration("antenna_offset_deg"),
                }
            ],
            output="screen",
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_file],
            remappings=[
                ('gps/fix', '/gnss/fix'),
                ('imu', '/gnss/heading'),
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',

            output='screen',

            parameters=[ekf_file]
        )
    ])