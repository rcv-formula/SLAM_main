import os
from datetime import datetime

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('cartographer_ros')
    config_dir = os.path.join(package_dir, 'configuration_files')
    package_prefix = get_package_prefix('cartographer_ros')
    workspace_dir = os.path.dirname(os.path.dirname(package_prefix))
    metrics_dir = os.path.join(workspace_dir, 'local_quality_metrics')
    os.makedirs(metrics_dir, exist_ok=True)
    local_quality_metrics_csv_path = os.path.join(
        metrics_dir,
        f"0501_local_quality_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    score_distribution_dir = os.path.join(
        workspace_dir, 'global_constraint_score_distributions')
    os.makedirs(score_distribution_dir, exist_ok=True)
    score_distribution_csv_path = os.path.join(
        score_distribution_dir,
        f"0501_fast_correlative_score_distribution_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    pbstream_file = LaunchConfiguration('pbstream_file')
    bagfiles = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fusion_extrapolator = LaunchConfiguration('fusion_extrapolator')
    pose_extrapolator_config = LaunchConfiguration('pose_extrapolator_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bagfiles',
            default_value='/home/symoon/Desktop/bag/0501/0501_DATA.bag',
            description='Path to the 0501 rosbag directory or db3 file'
        ),
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='/home/symoon/Desktop/bag/0501/0501.pbstream',
            description='Path to the 0501 pbstream file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        # true일 경우 scan imu fusion 기반 로직, false일 경우 원본 fusion 로직
        DeclareLaunchArgument(
            'fusion_extrapolator',
            default_value='true',
            description='Enable fusion-based extrapolator when true'
        ),
        DeclareLaunchArgument(
            'pose_extrapolator_config',
            default_value=os.path.join(workspace_dir, 'config.yaml'),
            description='Path to metric tuning YAML'
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
        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            additional_env={
                'LOCAL_QUALITY_METRICS_CSV_PATH': local_quality_metrics_csv_path,
                'FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH':
                    score_distribution_csv_path,
            },
            arguments=[
                '--collect_metrics',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_localization_config_wheel.lua',
                '-load_state_filename', pbstream_file,
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),  # 데이터 스트림 이름 일치 확인 필요
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
                {"provide_odom_frame": True},
                {"use_odometry": True},
                {"publish_frame_projected_to_2d": True}
            ],
        ),

        # Optional occupancy grid node for visualization of the map
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}],
            remappings=[
                ('map', 'map'),
                ('occupancy_grid', 'map'),
            ],
        ),

        # Optional node to transform trajectory to odom for robot localization
        Node(
            package='cartographer_ros',
            executable='trajectory_to_odom',
            name='trajectory_to_odom_node',
            output='screen',
            parameters=[
               {"use_sim_time": use_sim_time},
            ]
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bagfiles, '--clock'],
            output='screen',
        ),
    ])#
