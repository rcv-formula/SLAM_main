import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    default_pbstream_file = os.path.join(package_dir, 'pbstream/latest.pbstream')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fusion_extrapolator = LaunchConfiguration('fusion_extrapolator')
    pbstream_file = LaunchConfiguration('pbstream_file')
    use_occupancy_grid = LaunchConfiguration('use_occupancy_grid')
    pose_extrapolator_config = LaunchConfiguration('pose_extrapolator_config')
    use_initial_pose = LaunchConfiguration('use_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_relative_to_trajectory_id = LaunchConfiguration(
        'initial_pose_relative_to_trajectory_id')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        #true일 경우 scan imu fusion 사용, false일 경우 원본 
        DeclareLaunchArgument(
            'fusion_extrapolator',
            default_value='true',
            description='Enable fusion-based extrapolator when true'
        ),
        DeclareLaunchArgument(
            'pbstream_file',
            default_value=default_pbstream_file,
            description='Path to the frozen pbstream map for localization',
        ),
        DeclareLaunchArgument(
            'use_occupancy_grid',
            default_value='false',
            description='Publish occupancy grid map for visualization',
        ),
        DeclareLaunchArgument(
            'pose_extrapolator_config',
            default_value=os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(package_dir))),
                'config_mapping_wheel.yaml'),
            description='Path to wheel odom tuning YAML',
        ),
        DeclareLaunchArgument(
            'use_initial_pose',
            default_value='false',
            description='Start localization from an explicit map-frame pose',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose x relative to the frozen trajectory',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose y relative to the frozen trajectory',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw in radians relative to the frozen trajectory',
        ),
        DeclareLaunchArgument(
            'initial_pose_relative_to_trajectory_id',
            default_value='0',
            description='Frozen trajectory ID used as the initial-pose reference',
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
            arguments=[
                '-minloglevel', '1',
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_localization_config_wheel.lua',
                '-load_state_filename', pbstream_file,
                '-use_initial_pose', use_initial_pose,
                '-initial_pose_x', initial_pose_x,
                '-initial_pose_y', initial_pose_y,
                '-initial_pose_yaw', initial_pose_yaw,
                '-initial_pose_relative_to_trajectory_id',
                initial_pose_relative_to_trajectory_id,
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),  # 데이터 스트림 이름 일치 확인 필요
                ('odom_wheel', 'odom_wheel'),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
            ],
        ),

        # Optional occupancy grid node for visualization of the map
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            condition=IfCondition(use_occupancy_grid),
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
    ])
