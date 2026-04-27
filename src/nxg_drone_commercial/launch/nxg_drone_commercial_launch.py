import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction

def generate_launch_description():
    pkg_share_directory = get_package_share_directory('nxg_drone')
    model_share_directory = get_package_share_directory('nxg_drone_description')
    openvins_share_directory = get_package_share_directory('ov_msckf')
    model_sdf_file = os.path.join(model_share_directory, 'models', 'x500_commercial', 'model.sdf')
    config_path = os.path.join(openvins_share_directory, 'config', 'rs_d435i', 'estimator_config.yaml')
    with open(model_sdf_file, 'r') as fp:
        robot_description = fp.read()

    flight_control_node = TimerAction(
        period=15.0,
        actions=[
            Node (
                package='nxg_drone',
                executable='commercial',
                name='flight_control_node',
                output='both'
            )
        ]
    )

    target_detection_node = Node(
        package='nxg_drone',
        executable='TargetDetection',
        name='target_detection',
        output='log',
    )

    odometry_translation_node = Node(
        package='nxg_drone',
        executable='odometry_translate',
        name='px4_ros_odom_translate',
    )

    msckf_node = TimerAction(
        period=15.0,
        actions=[
            Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            namespace='ov_msckf',
            parameters = [
                {'use_stereo': True},
                {'max_cameras': 2},
                {'config_path': config_path},
                {'queue_size_imu': 20},
                {'queue_size_camera': 20},
            ],
            output='log',
            )
        ],
    )

    # ov_eval_node = Node(
    #     package='ov_eval',
    #     name='record_timing',
    #     executable='eval_timing',
    #     output='screen',
    #     parameters = [
    #         {'nodes': '/run_subscribe_msckf'},
    #         {'output': '/tmp/psutil_log.txt'},
    #     ]
    # )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        namespace='rplidar_a2m12',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'angle_compensate': True}],
        output='both'
    )

    translation_node = Node(
        package='nxg_drone',
        executable='px4_translation',
        name='px4_translation',
        output='screen'
    )

    realsense_share_directory = get_package_share_directory("realsense2_camera")
    depth_camera_launch_file = os.path.join(realsense_share_directory, "launch", "rs_launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            depth_camera_launch_file,
            launch_arguments={
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'gyro_fps': '400',
                'accel_fps': '400',
                'unite_imu_method': '2',
                'enable_color': 'true',
                'depth_module.infra_profile': '848x480x90',
            }.items(),
        ),
        ExecuteProcess(
            cmd=[['MicroXRCEAgent serial --dev /dev/ttyHS2 -b 921600']],
            shell=True,
            output='screen',
            name='micro_xrce_agent'
        ),
        flight_control_node,
        odometry_translation_node,
        msckf_node,
        # ov_eval_node,
        target_detection_node,
        rplidar_node,
        translation_node
    ])
