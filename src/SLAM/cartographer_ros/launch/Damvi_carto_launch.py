import os
from datetime import datetime

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('cartographer_ros')
    config_dir = os.path.join(package_share_dir, 'configuration_files')
    package_prefix = get_package_prefix('cartographer_ros')
    workspace_dir = os.path.dirname(os.path.dirname(package_prefix))
    metrics_dir = os.path.join(workspace_dir, 'cartographer_metrics')
    os.makedirs(metrics_dir, exist_ok=True)
    launch_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_odom_topic = LaunchConfiguration('cartographer_odom_topic')
    local_quality_metrics_csv_path = LaunchConfiguration(
        'local_quality_metrics_csv_path')
    fast_correlative_score_distribution_csv_path = LaunchConfiguration(
        'fast_correlative_score_distribution_csv_path')
    pose_graph_constraint_metrics_csv_path = LaunchConfiguration(
        'pose_graph_constraint_metrics_csv_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.',
        ),
        DeclareLaunchArgument(
            'cartographer_odom_topic',
            default_value='odometry/filtered',
            description='Topic remapped to Cartographer odom_wheel input.',
        ),
        DeclareLaunchArgument(
            'local_quality_metrics_csv_path',
            default_value=os.path.join(
                metrics_dir, f'local_quality_metrics_{launch_timestamp}.csv'),
            description='CSV path for local SLAM metrics.',
        ),
        DeclareLaunchArgument(
            'fast_correlative_score_distribution_csv_path',
            default_value=os.path.join(
                metrics_dir,
                f'fast_correlative_scores_{launch_timestamp}.csv'),
            description='CSV path for fast correlative score distributions.',
        ),
        DeclareLaunchArgument(
            'pose_graph_constraint_metrics_csv_path',
            default_value=os.path.join(
                metrics_dir,
                f'pose_graph_constraints_{launch_timestamp}.csv'),
            description='CSV path for pose graph constraint metrics.',
        ),
        SetEnvironmentVariable(
            name='LOCAL_QUALITY_METRICS_CSV_PATH',
            value=local_quality_metrics_csv_path,
        ),
        SetEnvironmentVariable(
            name='FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH',
            value=fast_correlative_score_distribution_csv_path,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_CONSTRAINT_METRICS_CSV_PATH',
            value=pose_graph_constraint_metrics_csv_path,
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
                '-configuration_basename', 'Damvi_carto_config.lua',
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),
                ('odom_wheel', cartographer_odom_topic),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
                ('odom', 'odom'),
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
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
