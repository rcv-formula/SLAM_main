import os
from datetime import datetime

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('cartographer_ros')
    config_dir = os.path.join(package_share_dir, 'configuration_files')
    package_prefix = get_package_prefix('cartographer_ros')
    workspace_dir = os.path.dirname(os.path.dirname(package_prefix))
    metrics_dir = os.path.join(workspace_dir, 'local_quality_metrics')
    os.makedirs(metrics_dir, exist_ok=True)
    metrics_csv_path = os.path.join(
        metrics_dir,
        f"local_quality_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    score_distribution_dir = os.path.join(
        workspace_dir, 'global_constraint_score_distributions')
    os.makedirs(score_distribution_dir, exist_ok=True)
    score_distribution_csv_path = os.path.join(
        score_distribution_dir,
        f"fast_correlative_score_distribution_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            additional_env={
                'LOCAL_QUALITY_METRICS_CSV_PATH': metrics_csv_path,
                'FAST_CORRELATIVE_SCORE_DISTRIBUTION_CSV_PATH':
                    score_distribution_csv_path,
            },
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--collect_metrics',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_carto_config.lua',
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'resolution': 0.05}],
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
            parameters=[{'use_sim_time': True}],
        ),
    ])
