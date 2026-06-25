"""
@File    swerve_drive.launch.py
@Time    2026/02/24
@Author  OpenArmX
@Version 1.0
@Desc    舵轮底盘主启动文件（ros2_control + 控制器 + RViz）
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 加载底盘运动学参数
    _desc_dir = get_package_share_directory('swerve_description')
    with open(os.path.join(_desc_dir, 'config', 'chassis_version_6.0.yaml')) as f:
        _cp = yaml.safe_load(f)['chassis']
    _wheel_radius   = str(_cp['wheel_radius'])
    _half_wheelbase = str(_cp['wheelbase'] / 2.0)
    _half_track     = str(_cp['track_width'] / 2.0)

    # 参数声明
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    steering_can_arg = DeclareLaunchArgument('steering_can_interface', default_value='can5')
    driving_can_arg = DeclareLaunchArgument('driving_can_interface', default_value='can4')
    max_wheel_speed_arg = DeclareLaunchArgument(
        'max_wheel_speed', default_value='2.0',
        description='舵轮控制器轮速上限 (m/s)，用于独立底盘/遥控模式')
    wheel_accel_limit_arg = DeclareLaunchArgument(
        'wheel_accel_limit', default_value='1.2',
        description='舵轮控制器轮速变化率上限 (m/s^2)')

    # URDF
    urdf_file = PathJoinSubstitution([
        FindPackageShare('swerve_description'), 'urdf', 'swerve.urdf.xacro'
    ])

    robot_description = Command([
        'xacro ', urdf_file,
        ' steering_can_interface:=', LaunchConfiguration('steering_can_interface'),
        ' driving_can_interface:=', LaunchConfiguration('driving_can_interface'),
    ])

    # 控制器配置
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('swerve_bringup'), 'config', 'controllers.yaml'
    ])
    controllers_params = RewrittenYaml(
        source_file=controllers_yaml,
        param_rewrites={
            'max_wheel_speed': LaunchConfiguration('max_wheel_speed'),
            'wheel_accel_limit': LaunchConfiguration('wheel_accel_limit'),
            'wheel_radius': _wheel_radius,
            'fl_pos_x':  _half_wheelbase,
            'fl_pos_y':  _half_track,
            'fr_pos_x':  _half_wheelbase,
            'fr_pos_y': f'-{_half_track}',
            'bl_pos_x': f'-{_half_wheelbase}',
            'bl_pos_y':  _half_track,
            'br_pos_x': f'-{_half_wheelbase}',
            'br_pos_y': f'-{_half_track}',
        },
        convert_types=True,
    )

    # RViz 配置
    rviz_config = PathJoinSubstitution([
        FindPackageShare('swerve_description'), 'rviz', 'swerve.rviz'
    ])

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen',
    )

    # controller_manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controllers_params,
        ],
        output='screen',
    )

    # joint_state_broadcaster（等待 controller_manager 就绪后再 configure）
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
    )

    # swerve_drive_controller
    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
    )

    # 确保 joint_state_broadcaster 先启动，再启动 swerve_drive_controller
    delay_swerve_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner],
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        steering_can_arg,
        driving_can_arg,
        max_wheel_speed_arg,
        wheel_accel_limit_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_swerve_controller,
        rviz_node,
    ])
