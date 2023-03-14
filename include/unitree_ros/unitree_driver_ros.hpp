#ifndef UNITREE_DRIVER_ROS_HPP
#define UNITREE_DRIVER_ROS_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

class UnitreeDriverRos : public rclcpp::Node {
   private:
    // This connection is used for High level commands only
    UNITREE_LEGGED_SDK::UDP robotUDPConnection;
    UNITREE_LEGGED_SDK::HighCmd robotHighCmd;
    UNITREE_LEGGED_SDK::HighState robotHighState;

   private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;

   public:
    UnitreeDriverRos();
    ~UnitreeDriverRos();
};

#endif  // UNITREE_DRIVER_ROS_HPP
