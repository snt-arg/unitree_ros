#include "unitree_ros/unitree_driver_ros.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ctime>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/utilities.hpp>

#include "unitree_legged_sdk/comm.h"

using namespace std::chrono_literals;

UnitreeDriverRos::UnitreeDriverRos()
    : Node("unitree_driver_node"),
      robotUDPConnection(robotLocalPort,
                         sizeof(robotHighCmd),
                         sizeof(robotHighState),
                         false,
                         UNITREE_LEGGED_SDK::RecvEnum::nonBlock,
                         true) {
    readParams();

    robotUDPConnection.SetIpPort(robotIP.c_str(), robotTargetPort);
    robotUDPConnection.InitCmdData(robotHighCmd);

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopicName,
        10,
        std::bind(&UnitreeDriverRos::cmdVelCallback, this, std::placeholders::_1));

    imuPub = create_publisher<sensor_msgs::msg::Imu>(imuTopicName, 10);
    odomPub = create_publisher<nav_msgs::msg::Odometry>(odometryTopicName, 10);
    batteryStatePub =
        create_publisher<unitree_ros::msg::BmsState>(bmsStateTopicName, 10);

    robotStateTimer = create_wall_timer(
        2ms, std::bind(&UnitreeDriverRos::robotStateTimerCallback, this));

    cmdVelResetTimer = create_wall_timer(
        1ms, std::bind(&UnitreeDriverRos::cmdVelResetTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Unitree Driver Node is ready!");
}

void UnitreeDriverRos::readParams() {
    RCLCPP_INFO(get_logger(), "Reading ROS parameters...");

    // Robot params
    declare_parameter<std::string>("robot_ip", "192.168.12.1");
    declare_parameter<int>("robot_target_port", 8082);
    // --------------------------------------------------------
    get_parameter("robot_ip", robotIP);
    get_parameter("robot_target_port", robotTargetPort);

    // Topic Names
    declare_parameter<std::string>("cmd_vel_topic_name", "cmd_vel");
    declare_parameter<std::string>("imu_topic_name", "/imu");
    declare_parameter<std::string>("odometry_topic_name", "/odom");
    declare_parameter<std::string>("bms_state_topic_name", "/bms_state");
    // --------------------------------------------------------
    get_parameter("cmd_vel_topic_name", cmdVelTopicName);
    get_parameter("odometry_topic_name", odometryTopicName);
    get_parameter("imu_topic_name", imuTopicName);
    get_parameter("bms_state_topic_name", bmsStateTopicName);

    // Frame Ids
    declare_parameter<std::string>("imu_frame_id", "IMU");
    declare_parameter<std::string>("odometry_frame_id", "map");
    declare_parameter<std::string>("odometry_child_frame_id", "odom");
    // --------------------------------------------------------
    get_parameter("odometry_frame_id", odometryFrameId);
    get_parameter("odometry_child_frame_id", odometryChildFrameId);
    get_parameter("imu_frame_id", imuFrameId);

    RCLCPP_INFO(get_logger(), "Finished reading ROS parameters!");
}

void UnitreeDriverRos::robotStateTimerCallback() {
    // Receive state from robot
    robotUDPConnection.Send();
    robotUDPConnection.Recv();
    robotUDPConnection.GetRecv(robotHighState);

    rclcpp::Time now = this->get_clock()->now();

    auto imuStateMsg = generateImuMsg(robotHighState, now, imuFrameId);
    auto odometryStateMsg =
        generateOdometryMsg(robotHighState, now, odometryFrameId, odometryChildFrameId);
    auto batteryStateMsg = generateBatteryStateMsg(robotHighState);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "body";

    transform.transform.translation.x = odometryStateMsg.pose.pose.position.x;
    transform.transform.translation.y = odometryStateMsg.pose.pose.position.y;
    transform.transform.translation.z = odometryStateMsg.pose.pose.position.z;

    tf2::Quaternion q;
    q.setRPY(odometryStateMsg.pose.pose.orientation.x,
             odometryStateMsg.pose.pose.orientation.y,
             odometryStateMsg.pose.pose.orientation.z);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    odomPub->publish(odometryStateMsg);
    imuPub->publish(imuStateMsg);
    batteryStatePub->publish(batteryStateMsg);
    tf_broadcaster->sendTransform(transform);
}

void UnitreeDriverRos::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    robotHighCmd = rosMsg2Cmd(msg);
    robotUDPConnection.SetSend(robotHighCmd);
    robotUDPConnection.Send();

    prevCmdVelSent = clock.now();
}

void UnitreeDriverRos::cmdVelResetTimerCallback() {
    auto delta_t = clock.now() - prevCmdVelSent;
    if (delta_t >= 400ms && delta_t <= 402ms) {
        robotHighCmd.velocity[0] = 0;
        robotHighCmd.velocity[1] = 0;
        robotHighCmd.velocity[2] = 0;
        robotHighCmd.yawSpeed = 0;
        robotUDPConnection.SetSend(robotHighCmd);
        robotUDPConnection.Send();
    }
}
