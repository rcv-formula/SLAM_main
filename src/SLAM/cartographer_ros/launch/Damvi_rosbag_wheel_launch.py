import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.realpath(__file__)
    launch_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(launch_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    qos_overrides_path = os.path.join(config_dir, 'rosbag_play_qos_overrides.yaml')
    slam_main_dir = os.environ.get(
        'SLAM_MAIN_DIR',
        '/home/rcv/Documents/slam_wheel/SLAM_main',
    )

    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fusion_extrapolator = LaunchConfiguration('fusion_extrapolator')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bagfiles',
            default_value=os.path.join(slam_main_dir, '0518_2_humble'),
            description='Path to the rosbag directory or db3 file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true',
        ),
        DeclareLaunchArgument(
            'fusion_extrapolator',
            default_value='true',
            description='Enable fusion-based extrapolator when true',
        ),
        SetEnvironmentVariable(
            name='FUSION_EXTRPOLATOR',
            value=fusion_extrapolator,
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'play',
                rosbag_file,
                '--clock',
                '--qos-profile-overrides-path',
                qos_overrides_path,
            ],
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'provide_odom_frame': True},
                {'use_odometry': True},
                {'publish_frame_projected_to_2d': True},
            ],
            arguments=[
                '--collect_metrics',
                '-minloglevel', '1',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_carto_config_wheel.lua',
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),
                ('odom_wheel', 'odom_wheel'),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[
                {'resolution': 0.05},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('map', 'map'),
                ('occupancy_grid', 'map'),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='trajectory_to_odom',
            name='trajectory_to_odom',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
