#ifndef UNITREE_DRIVER_ROS_HPP
#define UNITREE_DRIVER_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_ros/conversion.hpp"

// Ros Messages
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_ros/msg/bms_state.hpp>

class UnitreeDriverRos : public rclcpp::Node {
    // Parameters
   private:
    std::string cmdVelTopicName = "/cmd_vel";
    std::string odometryTopicName = "/odom";
    std::string odometryFrameId = "Odometry";
    std::string imuTopicName = "/imu";
    std::string imuFrameId = "IMU";
    std::string bmsStateTopicName = "/bms_state";
    std::string robotIP = "192.168.12.1";
    int robotLocalPort = 8090;
    int robotTargetPort = 8082;

    // Unitree Related
   public:
    // UDP connection used for High level commands
    UNITREE_LEGGED_SDK::UDP robotUDPConnection;
    // Unitree High Command struct
    UNITREE_LEGGED_SDK::HighCmd robotHighCmd = {};
    // Unitree High State struct
    UNITREE_LEGGED_SDK::HighState robotHighState = {};

    // Publishers
   private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    rclcpp::Publisher<unitree_ros::msg::BmsState>::SharedPtr batteryStatePub;

    // Subscriptions
   private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;

    // Timers
   private:
    rclcpp::TimerBase::SharedPtr robotStateTimer;

   public:
    /**
     * @brief Constructor of the class UnitreeDriverRos
     */
    UnitreeDriverRos();

   private:
    /**
     * @brief Callback function which publishes the odometry and imu data
     */
    void robotStateTimerCallback();

    /**
     * @brief Callback method which receives velocity commands and controls the robot
     *
     * @params msg: Twist message
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Method which reads ros_params and updates the variables values.
     */
    void readParams();
};

#endif  // UNITREE_DRIVER_ROS_HPP
