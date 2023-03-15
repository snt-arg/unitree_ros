#include "unitree_ros/unitree_driver_ros.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include "unitree_legged_sdk/comm.h"
#include "unitree_ros/conversion.hpp"

using namespace std::chrono_literals;

UnitreeDriverRos::UnitreeDriverRos()
    : Node("unitree_driver_node"),
      robot_ip("192.168.12.1"),
      robot_local_port(8090),
      robot_target_port(8082),
      robotUDPConnection(UNITREE_LEGGED_SDK::HIGHLEVEL,
                         robot_local_port,
                         robot_ip.c_str(),
                         robot_target_port) {
    readParams();

    robotUDPConnection.SetIpPort(robot_ip.c_str(), robot_target_port);
    robotUDPConnection.InitCmdData(robotHighCmd);

    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_name,
        10,
        std::bind(&UnitreeDriverRos::cmdVelCallback, this, std::placeholders::_1));

    robotStateTimer = create_wall_timer(
        50ms, std::bind(&UnitreeDriverRos::robotStateTimerCallback, this));

    imuPub = create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, 10);
    odomPub = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_name, 10);
}

void UnitreeDriverRos::readParams() {
    RCLCPP_INFO(get_logger(), "Reading parameters...");

    declare_parameter<std::string>("robot_ip", "192.168.12.1");
    declare_parameter<int>("robot_local_port", 8090);
    declare_parameter<int>("robot_target_port", 8082);
    declare_parameter<std::string>("cmd_vel_topic_name", "cmd_vel");
    declare_parameter<std::string>("odometry_topic_name", "/odom");
    declare_parameter<std::string>("imu_topic_name", "/imu");

    get_parameter("robot_ip", robot_ip);
    get_parameter("robot_local_port", robot_local_port);
    get_parameter("robot_target_port", robot_target_port);
    get_parameter("cmd_vel_topic_name", cmd_vel_topic_name);
    get_parameter("odometry_topic_name", odometry_topic_name);
    get_parameter("imu_topic_name", imu_topic_name);
    RCLCPP_INFO(get_logger(), "topic %s", cmd_vel_topic_name.c_str());

    RCLCPP_INFO(get_logger(), "Finished reading parameters!");
}

void UnitreeDriverRos::robotStateTimerCallback() {
    // Receive state from robot
    robotUDPConnection.Recv();
    robotUDPConnection.GetRecv(robotHighState);

    sensor_msgs::msg::Imu imuState;
    nav_msgs::msg::Odometry odometryState;
    rclcpp::Time now = this->get_clock()->now();

    // TODO: missing the frame_id
    imuState.header.stamp = now;
    imuState.orientation.x = robotHighState.imu.quaternion[0];
    imuState.orientation.y = robotHighState.imu.quaternion[1];
    imuState.orientation.z = robotHighState.imu.quaternion[2];
    imuState.orientation.w = robotHighState.imu.quaternion[3];
    imuState.linear_acceleration.x = robotHighState.imu.accelerometer[0];
    imuState.linear_acceleration.y = robotHighState.imu.accelerometer[1];
    imuState.linear_acceleration.z = robotHighState.imu.accelerometer[2];
    imuState.angular_velocity.x = robotHighState.imu.gyroscope[0];
    imuState.angular_velocity.y = robotHighState.imu.gyroscope[1];
    imuState.angular_velocity.z = robotHighState.imu.gyroscope[2];

    // TODO: missing the frame_id
    odometryState.header.stamp = now;
    odometryState.twist.twist.linear.x = robotHighState.velocity[0];
    odometryState.twist.twist.linear.y = robotHighState.velocity[1];
    odometryState.twist.twist.linear.z = robotHighState.velocity[2];
    odometryState.twist.twist.angular.z = robotHighState.yawSpeed;
    odometryState.pose.pose.position.x = robotHighState.position[0];
    odometryState.pose.pose.position.y = robotHighState.position[1];
    odometryState.pose.pose.position.z = robotHighState.position[2];
    odometryState.pose.pose.orientation.x = robotHighState.imu.quaternion[0];
    odometryState.pose.pose.orientation.y = robotHighState.imu.quaternion[1];
    odometryState.pose.pose.orientation.z = robotHighState.imu.quaternion[2];
    odometryState.pose.pose.orientation.w = robotHighState.imu.quaternion[3];

    odomPub->publish(odometryState);
    imuPub->publish(imuState);
}

void UnitreeDriverRos::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    robotHighCmd = rosMsg2Cmd(msg);
    robotUDPConnection.SetSend(robotHighCmd);
    robotUDPConnection.Send();
}
