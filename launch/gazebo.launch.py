import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                            ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_path = get_package_share_directory('hospital_robot_description')
    urdf_file  = os.path.join(pkg_path, 'urdf',   'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds',  'hospital.world')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # --- 1. Start Gazebo with hospital world ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file,
                          'verbose': 'true'}.items()
    )

    # --- 2. Robot state publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # --- 3. Spawn robot — delayed 5 seconds to let Gazebo fully start ---
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'hospital_robot',
            '-x', '0.0',   # center of corridor
            '-y', '0.0',
            '-z', '0.5',   # higher z so it doesn't spawn underground
            '-Y', '0.0'
        ],
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])