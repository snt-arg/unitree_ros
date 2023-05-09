#include "unitree_ros/unitree_driver_ros.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ctime>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/utilities.hpp>

#include "unitree_legged_sdk/comm.h"
#include "unitree_ros/conversion.hpp"
#include "unitree_ros/utils.hpp"

using namespace std::chrono_literals;

UnitreeDriverRos::UnitreeDriverRos()
    : Node("unitree_driver_node"),
      robotUDPConnection(robotLocalPort,
                         sizeof(robotHighCmd),
                         sizeof(robotHighState),
                         false,
                         UNITREE_LEGGED_SDK::RecvEnum::nonBlock,
                         true) {
    robotUDPConnection.SetIpPort(robotIP.c_str(), robotTargetPort);
    robotUDPConnection.InitCmdData(robotHighCmd);

    readParams();
    initPublishers();
    initSubscriptions();
    initTimers();

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Unitree Driver Node is ready!");
}

void UnitreeDriverRos::readParams() {
    RCLCPP_INFO(get_logger(), "Reading ROS parameters...");

    // Robot params
    declare_parameter<std::string>("robot_ip", robotIP);
    declare_parameter<int>("robot_target_port", robotTargetPort);
    // --------------------------------------------------------
    get_parameter("robot_ip", robotIP);
    get_parameter("robot_target_port", robotTargetPort);

    // Topic Names
    declare_parameter<std::string>("cmd_vel_topic_name", cmdVelTopicName);
    declare_parameter<std::string>("imu_topic_name", imuTopicName);
    declare_parameter<std::string>("odometry_topic_name", odometryTopicName);
    declare_parameter<std::string>("bms_state_topic_name", bmsStateTopicName);
    // --------------------------------------------------------
    get_parameter("cmd_vel_topic_name", cmdVelTopicName);
    get_parameter("odometry_topic_name", odometryTopicName);
    get_parameter("imu_topic_name", imuTopicName);
    get_parameter("bms_state_topic_name", bmsStateTopicName);

    // Frame Ids
    declare_parameter<std::string>("imu_frame_id", imuFrameId);
    declare_parameter<std::string>("odometry_frame_id", odometryFrameId);
    declare_parameter<std::string>("odometry_child_frame_id", odometryChildFrameId);
    // --------------------------------------------------------
    get_parameter("odometry_frame_id", odometryFrameId);
    get_parameter("odometry_child_frame_id", odometryChildFrameId);
    get_parameter("imu_frame_id", imuFrameId);

    // Flags
    declare_parameter<bool>("enable_obstacle_avoidance", enableObstacleAvoidance);

    RCLCPP_INFO(get_logger(), "Finished reading ROS parameters!");
}

void UnitreeDriverRos::initPublishers() {
    imuPub = create_publisher<sensor_msgs::msg::Imu>(imuTopicName, 10);
    odomPub = create_publisher<nav_msgs::msg::Odometry>(odometryTopicName, 10);
    batteryStatePub =
        create_publisher<unitree_ros::msg::BmsState>(bmsStateTopicName, 10);
}

void UnitreeDriverRos::initSubscriptions() {
    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopicName,
        10,
        std::bind(&UnitreeDriverRos::cmdVelCallback, this, std::placeholders::_1));
}

void UnitreeDriverRos::initTimers() {
    robotStateTimer = create_wall_timer(
        2ms, std::bind(&UnitreeDriverRos::robotStateTimerCallback, this));

    cmdVelResetTimer = create_wall_timer(
        1ms, std::bind(&UnitreeDriverRos::cmdVelResetTimerCallback, this));
}

void UnitreeDriverRos::robotStateTimerCallback() {
    // Receive state from robot
    robotUDPConnection.Send();
    robotUDPConnection.Recv();
    robotUDPConnection.GetRecv(robotHighState);

    rclcpp::Time now = this->get_clock()->now();

    auto imuStateMsg = convertIMUDataToMsg(robotHighState, now, imuFrameId);
    auto odometryStateMsg = convertOdometryDataToMsg(
        robotHighState, now, odometryFrameId, odometryChildFrameId);
    auto batteryStateMsg = convertBMSDataToMsg(robotHighState);
    auto odomTransform = generateOdomTransform(
        odometryStateMsg, now, odometryFrameId, odometryChildFrameId);

    odomPub->publish(odometryStateMsg);
    imuPub->publish(imuStateMsg);
    batteryStatePub->publish(batteryStateMsg);
    tf_broadcaster->sendTransform(odomTransform);
}

void UnitreeDriverRos::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    robotHighCmd = convertTwistMsgToHighLevelCmd(msg);

    // Check if the robot is in obstacle avoidance mode
    if (enableObstacleAvoidance) {
        float leftRange = robotHighState.rangeObstacle[0];
        float rightRange = robotHighState.rangeObstacle[1];
        float frontRange = robotHighState.rangeObstacle[2];

        if (isCollisionPossible(frontRange, leftRange, rightRange)) {
            if (msg->linear.x > 0 and frontRange < 0.4) {
                robotHighCmd.velocity[0] = 0;
            } else if (msg->linear.y > 0 and leftRange < 0.4) {
                robotHighCmd.velocity[1] = 0;
            } else if (msg->linear.y < 0 and rightRange < 0.4) {
                robotHighCmd.velocity[1] = 0;
            }
        }
    }
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
