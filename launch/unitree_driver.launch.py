import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("unitree_ros")
    param_file = os.path.join(pkg_dir, "config", "params.yaml")

    unitree_driver_node = Node(
        package="unitree_ros",
        executable="unitree_driver",
        parameters=[param_file],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(unitree_driver_node)

    return ld
