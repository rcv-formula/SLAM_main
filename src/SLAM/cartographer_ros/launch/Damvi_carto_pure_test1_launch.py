import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    default_pbstream_file = os.path.join(package_dir, 'pbstream', '0125_1.pbstream')
    pbstream_file = LaunchConfiguration('pbstream_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_odom_topic = LaunchConfiguration('cartographer_odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'pbstream_file',
            default_value=default_pbstream_file,
            description='Path to the pbstream file used for localization'
        ),
        DeclareLaunchArgument(
            'cartographer_odom_topic',
            default_value='odom_wheel',
            description='Topic remapped to Cartographer odom_wheel input'
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_localization_config_test1.lua',
                '-load_state_filename', pbstream_file
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
                {'publish_frame_projected_to_2d': True}
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
            name='trajectory_to_odom_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ]
        ),
    ])
