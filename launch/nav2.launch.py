import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path     = get_package_share_directory('hospital_robot_description')
    nav2_params  = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    map_file     = os.path.join(pkg_path, 'maps', 'hospital_map.yaml')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'map': map_file,
                        'params_file': nav2_params,
                        'use_sim_time': 'true',
                    }.items()
                ),
            ]
        )
    ])
