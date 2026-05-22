import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    score_distribution_dir = os.path.join(package_dir, 'global_constraint_score_distributions')
    os.makedirs(score_distribution_dir, exist_ok=True)
    score_distribution_csv_path = os.path.join(
        score_distribution_dir,
        f"fast_correlative_score_distribution_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    pbstream_file = LaunchConfiguration('pbstream_file')
    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'bagfiles',
            default_value='/home/rcv/Documents/SLAM_main/0507_sensor.bag_0.db3',
            description='Path to the rosbag file'
        ),
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='/home/rcv/Documents/SLAM_main/src/SLAM/cartographer_ros/pbstream/0501.pbstream',
            description='Path to the pbstream file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
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
                '-load_state_filename', pbstream_file 
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
                {"provide_odom_frame": True},
                {"use_odometry": False},
                {"publish_frame_projected_to_2d": True}
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[
                {'resolution': 0.05}
            
            ],
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
               {"use_sim_time": True},
            ]
        ),

        # ROS bag 재생 노드
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_file, '--clock'],
            output='screen'
        )
    ])
