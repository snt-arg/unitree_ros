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
    // Topic Names
   private:
    std::string cmdVelTopicName = "/cmd_vel";
    std::string odometryTopicName = "/odom";
    std::string imuTopicName = "/imu";
    std::string bmsStateTopicName = "/bms_state";

    // Frame Ids
   private:
    std::string odometryFrameId = "map";
    std::string odometryChildFrameId = "odom";
    std::string imuFrameId = "IMU";

    // Robot Related
   private:
    // UDP connection used for High level commands
    UNITREE_LEGGED_SDK::UDP robotUDPConnection;
    UNITREE_LEGGED_SDK::HighCmd robotHighCmd = {};
    UNITREE_LEGGED_SDK::HighState robotHighState = {};
    int robotLocalPort = 8090;
    int robotTargetPort = 8082;
    std::string robotIP = "192.168.12.1";

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
    rclcpp::TimerBase::SharedPtr cmdVelResetTimer;
    rclcpp::Time prevCmdVelSent;

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
     * @brief Callback function which sends an empty High command to the robot
     * in case no cmd vel was sent within a timeout
     */
    void cmdVelResetTimerCallback();

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
