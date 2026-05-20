import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    scripts_dir = os.path.join(package_dir, 'scripts')
    score_distribution_dir = os.path.join(package_dir, 'global_constraint_score_distributions')
    os.makedirs(score_distribution_dir, exist_ok=True)
    score_distribution_csv_path = os.path.join(
        score_distribution_dir,
        f"fast_correlative_score_distribution_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )

    pbstream_file = '/home/cartographer/SLAM_local-loss/src/SLAM/cartographer_ros/pbstream/0508.pbstream'
    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_odom_topic = LaunchConfiguration('cartographer_odom_topic')
    fault_start_sec = LaunchConfiguration('fault_start_sec')
    fault_duration_sec = LaunchConfiguration('fault_duration_sec')
    fault_mode = LaunchConfiguration('fault_mode')
    shift_fraction = LaunchConfiguration('shift_fraction')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bagfiles',
            default_value='/home/cartographer/SLAM_local-loss/0507_sensor.bag_0.db3',
            description='Path to the rosbag file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'cartographer_odom_topic',
            default_value='odom_wheel',
            description='Topic remapped to Cartographer odom_wheel input',
        ),
        DeclareLaunchArgument(
            'fault_start_sec',
            default_value='20.0',
            description='Bag-relative time when scan corruption starts'
        ),
        DeclareLaunchArgument(
            'fault_duration_sec',
            default_value='8.0',
            description='Duration of scan corruption'
        ),
        DeclareLaunchArgument(
            'fault_mode',
            default_value='shift',
            description='One of: shift, reverse, invalid, noise, sparse_noise, drop'
        ),
        DeclareLaunchArgument(
            'shift_fraction',
            default_value='0.5',
            description='Fraction of ranges to rotate in shift mode'
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            additional_env={
                'FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH':
                    score_distribution_csv_path,
            },
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_localization_config.lua',
                '-load_state_filename', pbstream_file,
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),
                ('odom_wheel', cartographer_odom_topic),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'provide_odom_frame': True},
                {'use_odometry': True},
                {'publish_frame_projected_to_2d': True},
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
            name='trajectory_to_odom_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        ),

        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(scripts_dir, 'relocalization_scan_fault_injector.py'),
                '--ros-args',
                '-p', 'input_topic:=/scan_fault_in',
                '-p', 'output_topic:=/scan',
                '-p', ['fault_start_sec:=', fault_start_sec],
                '-p', ['fault_duration_sec:=', fault_duration_sec],
                '-p', ['mode:=', fault_mode],
                '-p', ['shift_fraction:=', shift_fraction],
            ],
            output='screen',
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', rosbag_file, '--clock',
                '--remap', '/scan:=/scan_fault_in',
            ],
            output='screen',
        ),
    ])
