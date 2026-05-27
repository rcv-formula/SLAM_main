import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(launch_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    scripts_dir = os.path.join(package_dir, 'scripts')
    default_pbstream_file = os.path.join(package_dir, 'pbstream/latest.pbstream')

    pbstream_file = LaunchConfiguration('pbstream_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_initial_pose = LaunchConfiguration('use_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_relative_to_trajectory_id = LaunchConfiguration(
        'initial_pose_relative_to_trajectory_id')
    fault_start_sec = LaunchConfiguration('fault_start_sec')
    fault_duration_sec = LaunchConfiguration('fault_duration_sec')
    fault_mode = LaunchConfiguration('fault_mode')
    shift_fraction = LaunchConfiguration('shift_fraction')
    occlusion_fraction = LaunchConfiguration('occlusion_fraction')
    occlusion_distance = LaunchConfiguration('occlusion_distance')
    enable_scan_fault = LaunchConfiguration('enable_scan_fault')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true',
        ),
        DeclareLaunchArgument(
            'pbstream_file',
            default_value=default_pbstream_file,
            description='Path to the frozen pbstream map for localization',
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
        DeclareLaunchArgument(
            'enable_scan_fault',
            default_value='false',
            description='Route /scan through the scan fault injector when true',
        ),
        DeclareLaunchArgument(
            'fault_start_sec',
            default_value='20.0',
            description='Bag-relative time when scan corruption starts',
        ),
        DeclareLaunchArgument(
            'fault_duration_sec',
            default_value='0.0',
            description='Duration of scan corruption',
        ),
        DeclareLaunchArgument(
            'fault_mode',
            default_value='freeze',
            description='One of: freeze, front_occlusion, drop, invalid, noise, sparse_noise, shift, reverse',
        ),
        DeclareLaunchArgument(
            'shift_fraction',
            default_value='0.5',
            description='Fraction of ranges to rotate in shift mode',
        ),
        DeclareLaunchArgument(
            'occlusion_fraction',
            default_value='0.35',
            description='Centered scan fraction clamped in front_occlusion mode',
        ),
        DeclareLaunchArgument(
            'occlusion_distance',
            default_value='0.35',
            description='Synthetic obstacle distance in front_occlusion mode',
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '--collect_metrics',
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
            name='trajectory_to_odom_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        ),

        ExecuteProcess(
            condition=IfCondition(enable_scan_fault),
            cmd=[
                'python3',
                os.path.join(scripts_dir, 'relocalization_scan_fault_injector.py'),
                '--ros-args',
                '-p', 'input_topic:=/scan_fault_in',
                '-p', 'output_topic:=/scan',
                '-p', ['fault_start_sec:=', fault_start_sec],
                '-p', ['fault_duration_sec:=', fault_duration_sec],
                '-p', ['mode:=', fault_mode],
                '-p', ['shift_fraction:=', shift_fraction],
                '-p', ['occlusion_fraction:=', occlusion_fraction],
                '-p', ['occlusion_distance:=', occlusion_distance],
            ],
            output='screen',
        ),
    ])
