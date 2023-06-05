#ifndef UNITREE_ROS_HPP
#define UNITREE_ROS_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <unitree_ros/msg/bms_state.hpp>

#include "unitree_ros/unitree_data.hpp"
#include "unitree_ros/unitree_driver.hpp"

class UnitreeRosNode : public rclcpp::Node {
   private:
    // Robot
    UnitreeDriver unitree_driver;
    std::string robot_ip = "";
    int robot_target_port = 0;

    // Topic Names
    std::string ns = "";  // namespace
    std::string cmd_vel_topic_name = ns + "/cmd_vel";
    std::string odom_topic_name = ns + "/odom";
    std::string imu_topic_name = ns + "/imu";
    std::string bms_topic_name = ns + "/bms";

    // Frame Ids
    std::string odom_frame_id = ns + "odom";
    std::string odom_child_frame_id = ns + "base_link";
    std::string imu_frame_id = ns + "imu";

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<unitree_ros::msg::BmsState>::SharedPtr bms_pub;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stand_up_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stand_down_sub;

    // Timers
    rclcpp::TimerBase::SharedPtr robot_state_timer;
    rclcpp::TimerBase::SharedPtr cmd_vel_reset_timer;
    rclcpp::Time prev_cmd_vel_sent;
    rclcpp::Clock clock;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

   public:
    UnitreeRosNode();
    ~UnitreeRosNode();

   private:
    void read_parameters();
    void init_subscriptions();
    void init_publishers();
    void init_timers();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::UniquePtr msg);
    void robot_state_callback();
    void cmd_vel_reset_callback();
    void stand_up_callback(const std_msgs::msg::Empty::UniquePtr msg);
    void stand_down_callback(const std_msgs::msg::Empty::UniquePtr msg);

    void publish_odom(rclcpp::Time time);
    void publish_imu(rclcpp::Time time);
    void publish_bms();
    void publish_odom_tf(rclcpp::Time time, odom_t odom);
    void publish_remote();

    void apply_namespace_to_topic_names();
};

#endif  // !UNITREE_ROS_HPP
