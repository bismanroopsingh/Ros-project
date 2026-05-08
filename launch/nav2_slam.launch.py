import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_path   = get_package_share_directory('hospital_robot_description')
    urdf_file  = os.path.join(pkg_path, 'urdf',   'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds',  'hospital.world')
    nav2_pkg   = get_package_share_directory('nav2_bringup')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # 1. Mesh path
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..')
    )

    # 2. Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 3. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 4. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/hospital_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[('/model/hospital_robot/tf', '/tf')],
        output='screen'
    )

    # 5. Odom TF publisher
    odom_tf_publisher = Node(
        package='hospital_robot_description',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6. Spawn robot
    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name',  'hospital_robot',
                    '-x', '0.0',
                    '-y', '-2.9',
                    '-z', '0.15',
                ],
                output='screen'
            )
        ]
    )

    # 7. SLAM Toolbox
    slam_toolbox = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[{
                    'use_sim_time':            True,
                    'odom_frame':              'odom',
                    'map_frame':               'map',
                    'base_frame':              'base_link',
                    'scan_topic':              '/scan',
                    'mode':                    'mapping',
                    'resolution':              0.05,
                    'transform_timeout':       0.5,
                    'tf_buffer_duration':      30.0,
                    'minimum_travel_distance': 0.3,
                    'minimum_travel_heading':  0.2,
                    'do_loop_closure':         True,
                }],
                output='screen'
            )
        ]
    )

    # 8. RViz to see the map being built
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_ign_resource_path,
        gz_sim,
        robot_state_publisher,
        bridge,
        odom_tf_publisher,
        spawn_robot,
        slam_toolbox,
        rviz,
    ])
