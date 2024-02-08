import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution
from launch.actions import OpaqueFunction


def generate_launch_description():
    pkg_dir = get_package_share_directory("unitree_ros")
    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=str(default_param_file)
    )
    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value=str(""))

    return LaunchDescription(
        [params_file_arg, robot_ip_arg, launch_unitree_driver()]
    )


def launch_unitree_driver():
    params_file = LaunchConfiguration("params_file")
    robot_ip = LaunchConfiguration("robot_ip")

    unitree_driver_node = Node(
        package="unitree_ros",
        executable="unitree_driver",
        parameters=[params_file, {"robot_ip": robot_ip}],
        output="screen",
        condition=IfCondition(NotEqualsSubstitution(robot_ip, "")),
    )

    return unitree_driver_node
