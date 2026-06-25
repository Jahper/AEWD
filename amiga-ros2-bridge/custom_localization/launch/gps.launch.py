from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os

# For the localization stack we have this structure:

#  node:lg580p_driver----------------------------
#         |                                     |
#  /gnss/fix (longitude, latitude)     /gnss/heading (yaw as IMU topic)
#         |-------------------------------------|                                     
#         ||                                    |
#  node:navsat_transform                        |                          
#         |                                     |
#  /odometry/gps (GPS odometry)                 |
#         |-------------------------------------|
#         ||              
#  node:ekf_node
#         |
#  /odometry/filtered (fused odometry from GPS and "IMU" heading)
#         |
#  [nav2 nodes]      

# There is also code for the IMU here. But due to time constraints we did not manage to get the IMU working. 
# The IMU code is commented out in this launch file. and not present in the architecture above. 

# Suggested improvements:
# - Fuse (working) IMU and GPS data in the ekf_node. A code block should already be ready for that in ekf.yaml.
# - A lot of arguments for the gps driver are currently launch arguments. Those are better placed in a config file.
# - The GPS driver currently uploads its heading topic with a maxed out covariance. This will override the (real) IMU topic even if the IMU is more accurate. 
#   Update the code for the node so that covariance is calculated with the heading accuracy field.
# - A known problem is that the first gps heading sensor data gets used to initialise the odom. And decides for the rest of the program lifetime that that direction is forward. 
#   And all following heading data is just used as +/- offsets to the forward direction. This is not a problem usually. But the first sensor read is often quite wrong.
#   Therefore we have disabled any heading data that is accurate (floating) from even publishing permanently . This is suboptimal. Because even semi accurate data gets ignored.
#   It is worth investigating if its possible for Odom to only initialise after a highly accurate heading topic has been published. Rather then disabling accurate heading data permanently .

def generate_launch_description():

    pkg_dir = get_package_share_directory('custom_localization')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')

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
        # GPS arguments
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

        # IMU arguments
        DeclareLaunchArgument(
            "imu_port",
            default_value="/dev/ttyUSB0"
        ),
         DeclareLaunchArgument(
            "imu_baud",
            default_value="115200"
        ),
        
        
        ## IMU specific. enable this once IMU is fixed.
        # ExecuteProcess(
        # cmd=[
        #     'ros2', 'run',
        #     'micro_ros_agent', 'micro_ros_agent',
        #     'serial',
        #     '--dev', imu_port,
        #     '-b', imu_baud,
        # ],
        # output='screen'
        # ),

        # GPS, does longitude, latitude, and heading calculations, and publishes them to /gnss/fix and /gnss/heading
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

        # Navsat transform node and EKF node for sensor fusion
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