#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>

// Ros Messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_ros/msg/bms_cmd.hpp>
#include <unitree_ros/msg/bms_state.hpp>
#include <unitree_ros/msg/high_cmd.hpp>
#include <unitree_ros/msg/high_state.hpp>

#include "unitree_ros/msg/detail/high_state__struct.hpp"

/**
 * @brief Update the high level command from a Twist message to send to robot
 *
 * @param msg: Twist message
 *
 * @return updated High level command
 */
inline UNITREE_LEGGED_SDK::HighCmd convertTwistMsgToHighLevelCmd(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = msg->linear.x;
    cmd.velocity[1] = msg->linear.y;
    cmd.yawSpeed = msg->angular.z;
    cmd.reserve = 0;
    cmd.mode = 2;
    cmd.gaitType = 1;

    return cmd;
}

/**
 * @brief Create an IMU message from the robot high state
 *
 * @param highState: high state struct received from robot
 * @param now: time stamp of the message
 * @param frame_id: frame id of the message
 *
 * @return imu message
 */
inline sensor_msgs::msg::Imu convertIMUDataToMsg(
    UNITREE_LEGGED_SDK::HighState &highState,
    rclcpp::Time now,
    std::string frame_id) {
    sensor_msgs::msg::Imu imuStateMsg;

    imuStateMsg.header.frame_id = frame_id;
    imuStateMsg.header.stamp = now;

    imuStateMsg.orientation.x = highState.imu.quaternion[0];
    imuStateMsg.orientation.y = highState.imu.quaternion[1];
    imuStateMsg.orientation.z = highState.imu.quaternion[2];
    imuStateMsg.orientation.w = highState.imu.quaternion[3];

    imuStateMsg.linear_acceleration.x = highState.imu.accelerometer[0];
    imuStateMsg.linear_acceleration.y = highState.imu.accelerometer[1];
    imuStateMsg.linear_acceleration.z = highState.imu.accelerometer[2];

    imuStateMsg.angular_velocity.x = highState.imu.gyroscope[0];
    imuStateMsg.angular_velocity.y = highState.imu.gyroscope[1];
    imuStateMsg.angular_velocity.z = highState.imu.gyroscope[2];

    return imuStateMsg;
}

/**
 * @brief Create an odometry message from the robot high state
 *
 * @param highState: high state struct received from robot
 * @param now: time stamp of the message
 * @param frame_id: frame id of the message
 * @param childFrameId: child frame id of the message
 *
 * @return odometry message
 */
inline nav_msgs::msg::Odometry convertOdometryDataToMsg(
    UNITREE_LEGGED_SDK::HighState &highState,
    rclcpp::Time now,
    std::string frameId,
    std::string childFrameId) {
    nav_msgs::msg::Odometry odometryStateMsg;

    odometryStateMsg.child_frame_id = childFrameId;
    odometryStateMsg.header.frame_id = frameId;
    odometryStateMsg.header.stamp = now;

    odometryStateMsg.twist.twist.linear.x = highState.velocity[0];
    odometryStateMsg.twist.twist.linear.y = highState.velocity[1];
    odometryStateMsg.twist.twist.linear.z = highState.velocity[2];
    odometryStateMsg.twist.twist.angular.z = highState.yawSpeed;

    odometryStateMsg.pose.pose.position.x = highState.position[0];
    odometryStateMsg.pose.pose.position.y = highState.position[1];
    odometryStateMsg.pose.pose.position.z = highState.position[2];

    odometryStateMsg.pose.pose.orientation.x = highState.imu.rpy[0];
    odometryStateMsg.pose.pose.orientation.y = highState.imu.rpy[1];
    odometryStateMsg.pose.pose.orientation.z = highState.imu.rpy[2];
    odometryStateMsg.pose.pose.orientation.w = 1;

    return odometryStateMsg;
}

inline unitree_ros::msg::BmsState convertBMSDataToMsg(
    UNITREE_LEGGED_SDK::HighState &highState) {
    unitree_ros::msg::BmsState msg;

    msg.soc = highState.bms.SOC;
    msg.cycle = highState.bms.cycle;
    msg.bq_ntc = highState.bms.BQ_NTC;
    msg.current = highState.bms.current;
    msg.mcu_ntc = highState.bms.MCU_NTC;
    msg.cell_vol = highState.bms.cell_vol;
    msg.version_h = highState.bms.version_h;
    msg.version_l = highState.bms.version_l;
    msg.bms_status = highState.bms.bms_status;

    return msg;
}

inline geometry_msgs::msg::TransformStamped generateOdomTransform(
    nav_msgs::msg::Odometry odom,
    rclcpp::Time now,
    string frameId,
    string childFrameId) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = frameId;
    transform.child_frame_id = childFrameId;

    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;

    tf2::Quaternion q;
    q.setRPY(odom.pose.pose.orientation.x,
             odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    return transform;
}

#endif  // !CONVERSION_HPP
