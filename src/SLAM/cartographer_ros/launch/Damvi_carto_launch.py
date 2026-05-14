import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.realpath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    workspace_dir = os.path.abspath(os.path.join(package_dir, '..', '..', '..'))

    fusion_extrapolator = LaunchConfiguration('fusion_extrapolator')
    pose_extrapolator_config = LaunchConfiguration('pose_extrapolator_config')

    return LaunchDescription([
        DeclareLaunchArgument('fusion_extrapolator', default_value='true', description='Enable fusion-based extrapolator when true'),
        DeclareLaunchArgument(
            'pose_extrapolator_config',
            default_value=os.path.join(workspace_dir, 'config.yaml'),
            description='Path to wheel odom tuning YAML'),
        SetEnvironmentVariable(name='FUSION_EXTRPOLATOR', value=fusion_extrapolator),
        SetEnvironmentVariable(name='WHEEL_ODOM_CONFIG', value=pose_extrapolator_config),
        SetEnvironmentVariable(name='POSE_EXTRAPOLATOR_CONFIG', value=pose_extrapolator_config),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments = [
                '--collect_metrics',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_carto_config.lua'],
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
            parameters=[{'resolution': 0.05}],
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
        )
    ])
