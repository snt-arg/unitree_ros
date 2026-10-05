from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP address of the Unitree Go1 robot'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='8082',
        description='UDP port for robot communication'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='500.0',
        description='Publish rate in Hz for joint states and low state'
    )
    
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='joint_states',
        description='Topic name for joint states'
    )
    
    low_state_topic_arg = DeclareLaunchArgument(
        'low_state_topic',
        default_value='low_state',
        description='Topic name for low-level state'
    )

    low_cmd_topic_arg = DeclareLaunchArgument(
        'low_cmd_topic',
        default_value='low_cmd',
        description='Topic name for low-level commands'
    )

    lowlevel_driver_node = Node(
        package='unitree_ros',
        executable='unitree_lowlevel_driver',
        name='unitree_lowlevel_driver',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'low_state_topic': LaunchConfiguration('low_state_topic'),
            'low_cmd_topic': LaunchConfiguration('low_cmd_topic'),
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        publish_rate_arg,
        joint_states_topic_arg,
        low_state_topic_arg,
        low_cmd_topic_arg,
        lowlevel_driver_node,
    ])