from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


# Go1 joint limits [rad]: hip, thigh, calf. Thigh/calf pitch axes are negative.
JOINT_LIMITS = {
    'hip': (-1.047, 1.047),
    'thigh': (-0.663, 2.966),
    'calf': (-2.721, -0.837),
}
LEGS = ['FR', 'FL', 'RR', 'RL']


def generate_robot_description(robot_ip, robot_port, default_kp, default_kd):
    """Generate a minimal URDF with links, revolute joints and the ros2_control
    hardware block. Real <joint> elements are required because the controller
    manager imports joint limits from the URDF (enforce_command_limits)."""
    joint_xml = ""
    control_joint_xml = ""
    for leg in LEGS:
        prev_link = 'base'
        for i, (name, (lower, upper)) in enumerate(JOINT_LIMITS.items()):
            joint = f"{leg}_{name}_joint"
            link = f"{leg}_{name}_link"
            axis = '0 0 1' if name == 'hip' else '0 1 0'
            joint_xml += f"""
    <joint name="{joint}" type="revolute">
      <parent link="{prev_link}"/>
      <child link="{link}"/>
      <origin xyz="0 0 -0.02" rpy="0 0 0"/>
      <axis xyz="{axis}"/>
      <limit lower="{lower}" upper="{upper}" effort="33.5" velocity="25.0"/>
    </joint>
    <link name="{link}"/>"""
            control_joint_xml += f"""
    <joint name="{joint}">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>"""
            prev_link = link

    robot_description = f"""<?xml version="1.0"?>
<robot name="unitree_go1">
  <link name="base"/>
{joint_xml}
  <ros2_control name="UnitreeHardwareInterface" type="system">
    <hardware>
      <plugin>unitree_ros/UnitreeHardwareInterface</plugin>
      <param name="robot_ip">{robot_ip}</param>
      <param name="robot_port">{robot_port}</param>
      <param name="default_kp">{default_kp}</param>
      <param name="default_kd">{default_kd}</param>
    </hardware>
{control_joint_xml}
  </ros2_control>
</robot>"""

    return robot_description


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    robot_port = LaunchConfiguration('robot_port').perform(context)
    default_kp = LaunchConfiguration('default_kp').perform(context)
    default_kd = LaunchConfiguration('default_kd').perform(context)

    robot_description = generate_robot_description(
        robot_ip, robot_port, default_kp, default_kd)

    controller_config_path = os.path.join(
        get_package_share_directory('unitree_ros'),
        'config', 'unitree_ros_control.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }, controller_config_path],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller_spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return [robot_state_publisher, ros2_control_node, joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner]


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP address of the Unitree Go1 robot')

    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='8082',
        description='UDP port for robot communication')

    default_kp_arg = DeclareLaunchArgument(
        'default_kp',
        default_value='60.0',
        description='Default position gain (Kp)')

    default_kd_arg = DeclareLaunchArgument(
        'default_kd',
        default_value='3.0',
        description='Default velocity gain (Kd)')

    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        default_kp_arg,
        default_kd_arg,
        OpaqueFunction(function=launch_setup),
    ])