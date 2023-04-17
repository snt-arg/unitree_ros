import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("unitree_ros")
    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")
    params_file = LaunchConfiguration("params_file")
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=str(default_param_file)
    )

    unitree_driver_node = Node(
        package="unitree_ros",
        executable="unitree_driver",
        parameters=[params_file],
        output="screen",
    )

    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "body", "os_lidar"],
        output="screen",
    )

    return LaunchDescription(
        [params_file_arg, unitree_driver_node, static_transform_node]
    )
