/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#ifndef UNITREE_ROS_HPP
#define UNITREE_ROS_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <unitree_ros/msg/bms_state.hpp>
#include <unitree_ros/msg/sensor_ranges.hpp>

#include "unitree_ros/common_defines.hpp"
#include "unitree_ros/unitree_driver.hpp"

class UnitreeRosNode : public rclcpp::Node {
   private:
    // Robot
    std::unique_ptr<UnitreeDriver> unitree_driver_;
    std::string robot_ip_ = "192.168.123.161";
    int robot_target_port_ = 8082;

    // Topic Names
    std::string ns_ = "";  // namespace
    std::string cmd_vel_topic_name_ = ns_ + "/cmd_vel";
    std::string odom_topic_name_ = ns_ + "/odom";
    std::string imu_topic_name_ = ns_ + "/imu";
    std::string bms_topic_name_ = ns_ + "/bms";
    std::string sensor_ranges_topic_name = ns_ + "/sensor_ranges";

    // Frame Ids
    std::string odom_frame_id_ = ns_ + "odom";
    std::string odom_child_frame_id_ = ns_ + "base_link";
    std::string imu_frame_id_ = ns_ + "imu";

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<unitree_ros::msg::BmsState>::SharedPtr bms_pub_;
    rclcpp::Publisher<unitree_ros::msg::SensorRanges>::SharedPtr sensor_ranges_pub_;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stand_up_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stand_down_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr robot_state_timer_;
    // This timer is used to reset the velocity command to 0, which is a safety
    // measure.
    rclcpp::TimerBase::SharedPtr cmd_vel_reset_timer_;
    rclcpp::TimerBase::SharedPtr check_robot_battery_timer_;
    rclcpp::Time prev_cmd_vel_sent_;
    rclcpp::Clock clock_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Flags
    uint8_t low_batt_shutdown_threshold_ = 20;
    bool use_obstacle_avoidance_ = false;

   public:
    /**
     * @brief Constructor of the class UnitreeRosNode
     */
    UnitreeRosNode();
    /**
     * @brief deconstructor of the class UnitreeRosNode
     */
    ~UnitreeRosNode();

   private:
    /**
     * @brief Reads all the availabe ROS parameters and writes their values to
     * the appropriate attribues
     */
    void read_parameters_();

    void init_subscriptions_();
    void init_publishers_();
    void init_timers_();

    void cmd_vel_callback_(const geometry_msgs::msg::Twist::UniquePtr msg);
    void robot_state_callback_();
    void check_robot_battery_callback_();
    void cmd_vel_reset_callback_();
    void stand_up_callback_(const std_msgs::msg::Empty::UniquePtr msg);
    void stand_down_callback_(const std_msgs::msg::Empty::UniquePtr msg);

    void publish_odom_(rclcpp::Time time);
    void publish_imu_(rclcpp::Time time);
    void publish_bms_();
    void publish_sensor_ranges_();
    void publish_odom_tf_(rclcpp::Time time, odom_t odom);

    void apply_namespace_to_topic_names_();
};

#endif  // !UNITREE_ROS_HPP
