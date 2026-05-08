from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode', default_value='mapping',
        description='"mapping" to build a new map, "localization" to use saved map'
    )

    slam_params = {
        'use_sim_time':           LaunchConfiguration('use_sim_time'),
        'odom_frame':             'odom',
        'map_frame':              'map',
        'base_frame':             'base_link',
        'scan_topic':             '/scan',
        'mode':                   LaunchConfiguration('slam_mode'),

        # ── transform / timing ──────────────────────────────────
        'transform_timeout':      0.5,
        'tf_buffer_duration':     30.0,
        'scan_queue_size':        50,
        'minimum_time_interval':  0.2,   # min gap between processed scans (s)

        # ── map resolution / size ────────────────────────────────
        'resolution':             0.05,   # 5 cm per cell
        'map_start_at_dock':      True,

        # ── loop closure ─────────────────────────────────────────
        'do_loop_closure':        True,
        'loop_search_maximum_distance': 3.0,
        'loop_match_minimum_chain_size': 10,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_min_response_coarse':     0.35,
        'loop_match_min_response_fine':       0.45,

        # ── ICP / scan matching ───────────────────────────────────
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty':    1.0,
        'fine_search_angle_offset':  0.00349,
        'coarse_search_angle_offset': 0.349,
        'coarse_angle_resolution':   0.0349,
        'minimum_angle_penalty':     0.9,
        'minimum_distance_penalty':  0.5,
        'use_scan_matching':         True,
        'use_scan_barycenter':       True,
        'minimum_travel_distance':   0.3,   # rescan every 0.3 m
        'minimum_travel_heading':    0.2,   # rescan every 0.2 rad
        'scan_buffer_size':          10,
        'scan_buffer_maximum_scan_distance': 10.0,
        'link_match_minimum_response_fine': 0.1,
        'link_scan_maximum_distance': 1.5,
    }

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom'),
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_mode_arg,
        slam_node,
    ])