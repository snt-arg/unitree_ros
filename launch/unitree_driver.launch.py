import os

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def generate_launch_description():
    pkg_dir = get_package_share_directory("unitree_ros")
    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=str(default_param_file)
    )
    unitree_driver_function = OpaqueFunction(function=launch_unitree_driver)

    return LaunchDescription([params_file_arg, unitree_driver_function])


def launch_unitree_driver(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")

    with open(params_file.perform(context)) as f:
        params = yaml.safe_load(f)["unitree_driver_node"]["ros__parameters"]

    unitree_driver_node = Node(
        package="unitree_ros",
        executable="unitree_driver",
        parameters=[params_file],
        output="screen",
    )

    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "-0.02",
            "0.20",
            "0",
            "0",
            "0",
            "1",
            params["odometry_child_frame_id"],
            "os_sensor",
        ],
        output="screen",
    )

    return [unitree_driver_node, static_transform_node]
