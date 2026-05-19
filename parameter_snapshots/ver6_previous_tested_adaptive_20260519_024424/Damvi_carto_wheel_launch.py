import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    config_candidates = [
        os.path.join(os.getcwd(), 'config.yaml'),
        os.path.abspath(os.path.join(package_dir, '..', '..', '..', 'config.yaml')),
        os.path.abspath(os.path.join(package_dir, '..', '..', '..', '..', 'config.yaml')),
    ]
    pose_extrapolator_config = next(
        (path for path in config_candidates if os.path.exists(path)),
        config_candidates[0])
    use_sim_time = {'use_sim_time': True}
    return LaunchDescription([
        SetEnvironmentVariable(
            name='POSE_EXTRAPOLATOR_CONFIG',
            value=pose_extrapolator_config),
        SetEnvironmentVariable(
            name='WHEEL_ODOM_CONFIG',
            value=pose_extrapolator_config),
        SetEnvironmentVariable(
            name='MAPPING_ADAPTIVE_CONFIG',
            value=pose_extrapolator_config),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[use_sim_time],
            arguments = [
                '--collect_metrics',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_carto_config_wheel.lua'],
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
            parameters=[use_sim_time, {'resolution': 0.05}],
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
            parameters=[use_sim_time],
        )
    ])
