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
    pose_extrapolator_config = LaunchConfiguration('pose_extrapolator_config')
    wheel_odom_twist_only = LaunchConfiguration('wheel_odom_twist_only')
    wheel_odom_linear_scale = LaunchConfiguration('wheel_odom_linear_scale')
    local_quality_metrics_csv_path = LaunchConfiguration(
        'local_quality_metrics_csv_path')

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
        DeclareLaunchArgument(
            'pose_extrapolator_config',
            default_value=os.path.join(slam_main_dir, 'config_mapping_wheel.yaml'),
            description='Path to wheel odom tuning YAML',
        ),
        DeclareLaunchArgument(
            'wheel_odom_twist_only',
            default_value='false',
            description='Use /odom_wheel twist.linear.x as distance only, ignoring wheel odom pose/yaw',
        ),
        DeclareLaunchArgument(
            'wheel_odom_linear_scale',
            default_value='2.6',
            description='Calibration scale applied to odometry twist.linear.x when wheel_odom_twist_only is true.',
        ),
        DeclareLaunchArgument(
            'local_quality_metrics_csv_path',
            default_value='/tmp/cartographer_local_quality_metrics.csv',
            description='CSV path for local SLAM pose prediction vs estimate metrics',
        ),
        SetEnvironmentVariable(
            name='FUSION_EXTRPOLATOR',
            value=fusion_extrapolator,
        ),
        SetEnvironmentVariable(
            name='WHEEL_ODOM_CONFIG',
            value=pose_extrapolator_config,
        ),
        SetEnvironmentVariable(
            name='POSE_EXTRAPOLATOR_CONFIG',
            value=pose_extrapolator_config,
        ),
        SetEnvironmentVariable(
            name='WHEEL_ODOM_TWIST_ONLY',
            value=wheel_odom_twist_only,
        ),
        SetEnvironmentVariable(
            name='WHEEL_ODOM_LINEAR_SCALE',
            value=wheel_odom_linear_scale,
        ),
        SetEnvironmentVariable(
            name='LOCAL_QUALITY_METRICS_CSV_PATH',
            value=local_quality_metrics_csv_path,
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
