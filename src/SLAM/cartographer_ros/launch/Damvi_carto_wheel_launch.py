import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(launch_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
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
            name='trajectory_to_odom',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
