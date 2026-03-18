import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction

def generate_launch_description():
    pkg_share_directory = get_package_share_directory('nxg_drone')
    model_share_directory = get_package_share_directory('nxg_drone_description')
    openvins_share_directory = get_package_share_directory('ov_msckf')
    model_sdf_file = os.path.join(model_share_directory, 'models', 'x500_commercial', 'model.sdf')
    config_path = os.path.join(openvins_share_directory, 'config', 'rs_d435i', 'estimator_config.yaml')
    with open(model_sdf_file, 'r') as fp:
        robot_description = fp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description}
        ]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg_share_directory, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': os.path.join(pkg_share_directory, 'config', 'gz_nxg_drone_commercial_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }
        ],
    )

    flight_control_node = TimerAction(
        period=10.0,
        actions=[
            Node (
                package='nxg_drone',
                executable='commercial',
                name='flight_control_node',
                output='both',
                # prefix=['xterm -e'],
                # emulate_tty=True
            )
        ]
    )

    odometry_translation_node = Node(
        package='nxg_drone',
        executable='odometry_translate',
        name='px4_ros_odom_translate',
    )

    target_detection_node = Node(
        package='nxg_drone',
        executable='TargetDetection',
        name='target_detection'
        output='log',
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
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'enable_depth': 'false',
                'gyro_fps': '400',
                'accel_fps': '200',
                'unite_imu_method': '2',
                'enable_color': 'true',
                'depth_module.infra_profile': '848x480x90',
            }.items(),
        ),
        bridge_node,
        robot_state_publisher_node,
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        ExecuteProcess(
            cmd=[['MicroXRCEAgent udp4 -p 8888']],
            shell=True,
            output='screen',
            name='micro_xrce_agent'
        ),
        flight_control_node,
        robot_localization_node,
        odometry_translation_node,
        msckf_node,
        target_detection_node
    ])