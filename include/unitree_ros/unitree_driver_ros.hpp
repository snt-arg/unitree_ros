#ifndef UNITREE_DRIVER_ROS_HPP
#define UNITREE_DRIVER_ROS_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <unitree_ros/msg/high_cmd.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

class UnitreeDriverRos : public rclcpp::Node {
   private:
    std::string cmd_vel_topic_name;
    std::string odometry_topic_name;
    std::string imu_topic_name;
    std::string robot_ip;
    int robot_local_port, robot_target_port;

   private:
    // This connection is used for High level commands only
    UNITREE_LEGGED_SDK::UDP robotUDPConnection;
    // Unitree High Command struct
    UNITREE_LEGGED_SDK::HighCmd robotHighCmd;
    // Unitree High State struct
    UNITREE_LEGGED_SDK::HighState robotHighState;

   private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
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
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void readParams();
};

#endif  // UNITREE_DRIVER_ROS_HPP
