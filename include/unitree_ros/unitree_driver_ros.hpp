#ifndef UNITREE_DRIVER_ROS_HPP
#define UNITREE_DRIVER_ROS_HPP

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_ros/conversion.hpp"

// Ros Messages
#include <geometry_msgs/msg/transform_stamped.hpp>
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
    std::string odometryFrameId = "odom";
    std::string odometryChildFrameId = "body";
    std::string imuFrameId = "IMU";

    // Robot Related
   private:
    int robotLocalPort = 8090;
    int robotTargetPort = 8082;
    std::string robotIP = "192.168.12.1";
    // UDP connection used for High level commands
    UNITREE_LEGGED_SDK::UDP robotUDPConnection;
    UNITREE_LEGGED_SDK::HighCmd robotHighCmd = {};
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
    rclcpp::TimerBase::SharedPtr cmdVelResetTimer;
    rclcpp::Time prevCmdVelSent;
    rclcpp::Clock clock;

    // TF
   private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

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
