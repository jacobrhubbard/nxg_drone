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
        period=10.0,
        actions=[
            Node (
                package='nxg_drone',
                executable='commercial',
                name='flight_control_node',
                output='both'
            )
        ]
    )

    odometry_translation_node = Node(
        package='nxg_drone',
        executable='odometry_translate',
        name='px4_ros_odom_translate',
    )

    msckf_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        parameters = [
            {'use_stereo': True},
            {'max_cameras': 2},
            {'config_path': config_path}
        ],
        output='both'
    )

    realsense_share_directory = get_package_share_directory("realsense2_camera")
    depth_camera_launch_file = os.path.join(realsense_share_directory, "launch", "rs_launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            depth_camera_launch_file,
            launch_arguments={
                'enable_sync': 'true',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'enable_color': 'true',
                'enable_depth': 'true',
                'accel_fps': '200',
                'gyro_fps': '200',
            }.items(),
        ),
        ExecuteProcess(
            cmd=[['MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600']],
            shell=True,
            output='screen',
            name='micro_xrce_agent'
        ),
        flight_control_node,
        odometry_translation_node,
        msckf_node
    ])