import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    scripts_dir = os.path.join(package_dir, 'scripts')
    qos_overrides_path = os.path.join(config_dir, 'rosbag_play_qos_overrides.yaml')
    default_pose_extrapolator_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(package_dir))),
        'config_mapping_wheel.yaml',
    )
    score_distribution_dir = os.path.join(package_dir, 'global_constraint_score_distributions')
    os.makedirs(score_distribution_dir, exist_ok=True)
    score_distribution_csv_path = os.path.join(
        score_distribution_dir,
        f"fast_correlative_score_distribution_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    pose_graph_constraint_metrics_csv_path = os.path.join(
        score_distribution_dir,
        f"pose_graph_constraint_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    default_pbstream_file = os.path.join(package_dir, 'pbstream/latest.pbstream')

    pbstream_file = LaunchConfiguration('pbstream_file')
    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fusion_extrapolator = LaunchConfiguration('fusion_extrapolator')
    pose_extrapolator_config = LaunchConfiguration('pose_extrapolator_config')
    wheel_odom_twist_only = LaunchConfiguration('wheel_odom_twist_only')
    wheel_odom_linear_scale = LaunchConfiguration('wheel_odom_linear_scale')
    local_quality_metrics_csv_path = LaunchConfiguration(
        'local_quality_metrics_csv_path')
    pose_graph_constraint_metrics_csv = LaunchConfiguration(
        'pose_graph_constraint_metrics_csv_path')
    local_lateral_residual_max = LaunchConfiguration(
        'local_lateral_residual_max')
    clamp_local_lateral_residual = LaunchConfiguration(
        'clamp_local_lateral_residual')
    imu_yaw_weight = LaunchConfiguration('imu_yaw_weight')
    wheel_odom_yaw_weight = LaunchConfiguration('wheel_odom_yaw_weight')
    enable_relocalization_guards = LaunchConfiguration(
        'enable_relocalization_guards')
    enable_tracking_global_guards = LaunchConfiguration(
        'enable_tracking_global_guards')
    restart_on_lost = LaunchConfiguration('restart_on_lost')
    restart_lost_after_sec = LaunchConfiguration('restart_lost_after_sec')
    restart_cooldown_sec = LaunchConfiguration('restart_cooldown_sec')
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
            'bagfiles',
            default_value='/home/rcv/Documents/localization_wheel/SLAM_main/0525_humble',
            description='Path to the rosbag directory'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
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
            'pose_extrapolator_config',
            default_value=default_pose_extrapolator_config,
            description='Path to wheel odom tuning YAML'
        ),
        DeclareLaunchArgument(
            'wheel_odom_twist_only',
            default_value='true',
            description='Use /odom_wheel twist.linear.x as distance only'
        ),
        DeclareLaunchArgument(
            'wheel_odom_linear_scale',
            default_value='2.6',
            description='Calibration scale applied to wheel odom twist.linear.x'
        ),
        DeclareLaunchArgument(
            'local_quality_metrics_csv_path',
            default_value='/tmp/cartographer_localization_quality_metrics.csv',
            description='CSV path for local localization quality metrics',
        ),
        DeclareLaunchArgument(
            'pose_graph_constraint_metrics_csv_path',
            default_value=pose_graph_constraint_metrics_csv_path,
            description='CSV path for pose graph constraint yaw metrics',
        ),
        DeclareLaunchArgument(
            'local_lateral_residual_max',
            default_value='0.03',
            description='Maximum local scan-match lateral correction per scan in meters',
        ),
        DeclareLaunchArgument(
            'clamp_local_lateral_residual',
            default_value='false',
            description='Clamp local scan-match lateral correction when true',
        ),
        DeclareLaunchArgument(
            'imu_yaw_weight',
            default_value='0.45',
            description='Weight applied to IMU yaw integration in pose prediction',
        ),
        DeclareLaunchArgument(
            'wheel_odom_yaw_weight',
            default_value='0.0',
            description='Weight applied to wheel odometry angular.z',
        ),
        DeclareLaunchArgument(
            'enable_relocalization_guards',
            default_value='false',
            description='Enable experimental ambiguous-global-match rejection gates',
        ),
        DeclareLaunchArgument(
            'enable_tracking_global_guards',
            default_value='false',
            description='Bound normal tracking global constraints to the predicted pose',
        ),
        DeclareLaunchArgument(
            'restart_on_lost',
            default_value='false',
            description='Restart localization trajectory when LOST persists',
        ),
        DeclareLaunchArgument(
            'restart_lost_after_sec',
            default_value='3.0',
            description='Seconds of continuous LOST before trajectory restart',
        ),
        DeclareLaunchArgument(
            'restart_cooldown_sec',
            default_value='8.0',
            description='Minimum seconds between automatic restarts',
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
            description='Route /scan through the scan fault injector when true'
        ),
        DeclareLaunchArgument(
            'fault_start_sec',
            default_value='20.0',
            description='Bag-relative time when scan corruption starts'
        ),
        DeclareLaunchArgument(
            'fault_duration_sec',
            default_value='0.0',
            description='Duration of scan corruption'
        ),
        DeclareLaunchArgument(
            'fault_mode',
            default_value='freeze',
            description='One of: freeze, front_occlusion, drop, invalid, noise, sparse_noise, shift, reverse'
        ),
        DeclareLaunchArgument(
            'shift_fraction',
            default_value='0.5',
            description='Fraction of ranges to rotate in shift mode'
        ),
        DeclareLaunchArgument(
            'occlusion_fraction',
            default_value='0.35',
            description='Centered scan fraction clamped in front_occlusion mode'
        ),
        DeclareLaunchArgument(
            'occlusion_distance',
            default_value='0.35',
            description='Synthetic obstacle distance in front_occlusion mode'
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
        SetEnvironmentVariable(
            name='WHEEL_ODOM_TWIST_ONLY',
            value=wheel_odom_twist_only,
        ),
        SetEnvironmentVariable(
            name='WHEEL_ODOM_LINEAR_SCALE',
            value=wheel_odom_linear_scale,
        ),
        SetEnvironmentVariable(
            name='LOCAL_QUALITY_METRICS_CSV_PATH',
            value=local_quality_metrics_csv_path,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_CONSTRAINT_METRICS_CSV_PATH',
            value=pose_graph_constraint_metrics_csv,
        ),
        SetEnvironmentVariable(
            name='CARTOGRAPHER_CLAMP_LOCAL_LATERAL_RESIDUAL',
            value=clamp_local_lateral_residual,
        ),
        SetEnvironmentVariable(
            name='CARTOGRAPHER_LOCAL_LATERAL_RESIDUAL_MAX',
            value=local_lateral_residual_max,
        ),
        SetEnvironmentVariable(
            name='CARTOGRAPHER_IMU_YAW_WEIGHT',
            value=imu_yaw_weight,
        ),
        SetEnvironmentVariable(
            name='WHEEL_ODOM_YAW_WEIGHT',
            value=wheel_odom_yaw_weight,
        ),
        SetEnvironmentVariable(
            name='CARTOGRAPHER_RESTART_ON_LOCALIZATION_LOST',
            value=restart_on_lost,
        ),
        SetEnvironmentVariable(
            name='CARTOGRAPHER_RESTART_LOST_AFTER_SEC',
            value=restart_lost_after_sec,
        ),
        SetEnvironmentVariable(
            name='CARTOGRAPHER_RESTART_COOLDOWN_SEC',
            value=restart_cooldown_sec,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_CONSTRAINT_DOWNWEIGHT',
            value=enable_relocalization_guards,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_CONSTRAINT_REJECT',
            value=enable_relocalization_guards,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_APPLY_TO_TRACKING',
            value='false',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_APPLY_TO_RECOVERY',
            value=enable_relocalization_guards,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_CONSTRAINT_REJECT_MIN_SCORE',
            value='0.70',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MIN_TRANSLATION',
            value='1.0',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MAX_SCORE_MARGIN',
            value='0.001',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_CONSTRAINT_MIN_NEAR_TOP_COUNT',
            value='20',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_REJECT_AMBIGUOUS_FULL_SUBMAP',
            value=enable_relocalization_guards,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_FULL_SUBMAP_MAX_SCORE_MARGIN',
            value='0.001',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_FULL_SUBMAP_REJECT_MIN_SCORE',
            value='0.70',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_AMBIGUOUS_FULL_SUBMAP_MIN_NEAR_TOP_COUNT',
            value='80',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_BOUND_RELOCALIZATION_TO_PRIOR',
            value=enable_relocalization_guards,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_BOUND_TRACKING_GLOBAL_TO_PRIOR',
            value=enable_tracking_global_guards,
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_TRACKING_PRIOR_MIN_SCORE',
            value='0.70',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_TRACKING_MAX_TRANSLATION_CORRECTION',
            value='0.80',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_TRACKING_MAX_YAW_CORRECTION',
            value='0.45',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_RELOCALIZATION_PRIOR_MIN_SCORE',
            value='0.70',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_RELOCALIZATION_MAX_TRANSLATION_CORRECTION',
            value='1.5',
        ),
        SetEnvironmentVariable(
            name='POSE_GRAPH_RELOCALIZATION_MAX_YAW_CORRECTION',
            value='0.8',
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

        ExecuteProcess(
            condition=IfCondition(enable_scan_fault),
            cmd=[
                'ros2', 'bag', 'play', rosbag_file, '--clock',
                '--qos-profile-overrides-path', qos_overrides_path,
                '--topics', '/scan', '/imu/data', '/odom_wheel', '/tf', '/tf_static',
                '--remap', '/scan:=/scan_fault_in',
            ],
            output='screen',
        ),

        ExecuteProcess(
            condition=UnlessCondition(enable_scan_fault),
            cmd=[
                'ros2', 'bag', 'play', rosbag_file, '--clock',
                '--qos-profile-overrides-path', qos_overrides_path,
                '--topics', '/scan', '/imu/data', '/odom_wheel', '/tf', '/tf_static',
            ],
            output='screen',
        ),
    ])
