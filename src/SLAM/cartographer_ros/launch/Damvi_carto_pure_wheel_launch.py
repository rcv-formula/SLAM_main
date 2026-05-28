import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    metrics_dir = '/home/rcv/SLAM_main-SLAM_IMU_WHEEL_tun_upg/cartographer_metrics'
    os.makedirs(metrics_dir, exist_ok=True)

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-minloglevel', '1',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_localization_config_wheel.lua',
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),  # 데이터 스트림 이름 일치 확인 필요
                ('odom_wheel', 'odom_wheel'),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
        ),

        # Optional occupancy grid node for visualization of the map
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            remappings=[
                ('map', 'map'),
                ('occupancy_grid', 'map'),
            ],
        ),
    ])
