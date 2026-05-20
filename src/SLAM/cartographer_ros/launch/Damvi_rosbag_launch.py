import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(launch_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    qos_overrides_path = os.path.join(config_dir, 'rosbag_play_qos_overrides.yaml')
    default_ekf_config_path = os.path.join(
        config_dir, 'ekf_wheel_pose_delta_imu.yaml')

    slam_main_dir = os.environ.get(
        'SLAM_MAIN_DIR',
        '/home/rcv/Documents/slam_wheel/SLAM_main',
    )
    metrics_dir = os.path.join(slam_main_dir, 'cartographer_metrics')
    os.makedirs(metrics_dir, exist_ok=True)
    launch_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_config_path = LaunchConfiguration('ekf_config_path')
    cartographer_odom_topic = LaunchConfiguration('cartographer_odom_topic')
    local_quality_metrics_csv_path = LaunchConfiguration(
        'local_quality_metrics_csv_path')
    fast_correlative_score_distribution_csv_path = LaunchConfiguration(
        'fast_correlative_score_distribution_csv_path')
    pose_graph_constraint_metrics_csv_path = LaunchConfiguration(
        'pose_graph_constraint_metrics_csv_path')

    cartographer_node = Node(
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
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[
            ekf_config_path,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bagfiles',
            default_value=os.path.join(slam_main_dir, '0518_2_humble'),
            description='Path to the rosbag directory or db3 file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true.',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='Run robot_localization EKF for diagnostics.',
        ),
        DeclareLaunchArgument(
            'ekf_config_path',
            default_value=default_ekf_config_path,
            description='robot_localization EKF YAML path.',
        ),
        DeclareLaunchArgument(
            'cartographer_odom_topic',
            default_value='odom_wheel',
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
        ekf_node,
        cartographer_node,
        Node(
            package='cartographer_ros',
            executable='trajectory_to_odom',
            name='trajectory_to_odom',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
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
    ])
