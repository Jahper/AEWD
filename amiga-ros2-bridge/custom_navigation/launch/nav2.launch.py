from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory(
        'nav2_bringup'
    )

    custom_nav_dir = get_package_share_directory(
        'custom_navigation'
    )

    params_file = os.path.join(
        custom_nav_dir,
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup_dir,
                    'launch',
                    'bringup_launch.py'
                )
            ),

            launch_arguments={

                'slam': 'False',

                'use_sim_time': 'false',

                'map' : '',

                'params_file': params_file,

                'autostart': 'true',

                'use_composition': 'False'

            }.items()

        )

    ])