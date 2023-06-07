#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <unitree_ros/unitree_ros.hpp>

#include "unitree_ros/serializers.hpp"

UnitreeRosNode::UnitreeRosNode() : Node("unitree_ros_node"), unitree_driver() {
    read_parameters();
    init_subscriptions();
    init_publishers();
    init_timers();
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    unitree_driver.enable_obstacle_avoidance(use_obstacle_avoidance);

    RCLCPP_INFO(get_logger(), "Unitree ROS node initialized!");
}

UnitreeRosNode::~UnitreeRosNode() {
    RCLCPP_INFO(get_logger(), "Shutting down Unitree ROS node...");
}

void UnitreeRosNode::read_parameters() {
    RCLCPP_INFO(get_logger(), "Reading ROS parameters...");

    // Robot params
    declare_parameter<std::string>("robot_ip", robot_ip);
    declare_parameter<int>("robot_target_port", robot_target_port);
    // --------------------------------------------------------
    get_parameter("robot_ip", robot_ip);
    get_parameter("robot_target_port", robot_target_port);

    // Namespace
    declare_parameter<std::string>("ns", ns);
    // --------------------------------------------------------
    get_parameter("ns", ns);

    // Topic Names
    declare_parameter<std::string>("cmd_vel_topic_name", cmd_vel_topic_name);
    declare_parameter<std::string>("imu_topic_name", imu_topic_name);
    declare_parameter<std::string>("odom_topic_name", odom_topic_name);
    declare_parameter<std::string>("bms_state_topic_name", bms_topic_name);
    // --------------------------------------------------------
    get_parameter("cmd_vel_topic_name", cmd_vel_topic_name);
    get_parameter("odom_topic_name", odom_topic_name);
    get_parameter("imu_topic_name", imu_topic_name);
    get_parameter("bms_state_topic_name", bms_topic_name);

    // Frame Ids
    declare_parameter<std::string>("imu_frame_id", imu_frame_id);
    declare_parameter<std::string>("odom_frame_id", odom_frame_id);
    declare_parameter<std::string>("odom_child_frame_id", odom_child_frame_id);
    // --------------------------------------------------------
    get_parameter("odom_frame_id", odom_frame_id);
    get_parameter("odom_child_frame_id", odom_child_frame_id);
    get_parameter("imu_frame_id", imu_frame_id);

    // Flags
    declare_parameter<uint8_t>("low_batt_shutdown_threshold",
                               low_batt_shutdown_threshold);
    declare_parameter<bool>("use_obstacle_avoidance", use_obstacle_avoidance);
    // --------------------------------------------------------
    get_parameter("low_batt_shutdown_threshold", low_batt_shutdown_threshold);
    get_parameter("use_obstacle_avoidance", use_obstacle_avoidance);

    apply_namespace_to_topic_names();
    RCLCPP_INFO(get_logger(), "Finished reading ROS parameters!");
}

void UnitreeRosNode::apply_namespace_to_topic_names() {
    cmd_vel_topic_name = ns + cmd_vel_topic_name;
    odom_topic_name = ns + odom_topic_name;
    imu_topic_name = ns + imu_topic_name;
    bms_topic_name = ns + bms_topic_name;

    if (ns != "") {
        imu_frame_id = ns + '/' + imu_frame_id;
        odom_frame_id = ns + '/' + odom_frame_id;
        odom_child_frame_id = ns + '/' + odom_child_frame_id;
    }
}

void UnitreeRosNode::init_subscriptions() {
    RCLCPP_INFO(get_logger(), "Initializing ROS subscriptions...");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_name,
        qos,
        std::bind(&UnitreeRosNode::cmd_vel_callback, this, std::placeholders::_1));

    stand_up_sub = this->create_subscription<std_msgs::msg::Empty>(
        "stand_up",
        qos,
        std::bind(&UnitreeRosNode::stand_up_callback, this, std::placeholders::_1));

    stand_down_sub = this->create_subscription<std_msgs::msg::Empty>(
        "stand_down",
        qos,
        std::bind(&UnitreeRosNode::stand_down_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Finished initializing ROS subscriptions!");
}

void UnitreeRosNode::init_publishers() {
    RCLCPP_INFO(get_logger(), "Initializing ROS publishers...");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, qos);
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, qos);
    bms_pub = this->create_publisher<unitree_ros::msg::BmsState>(bms_topic_name, qos);

    RCLCPP_INFO(get_logger(), "Finished initializing ROS publishers!");
}

void UnitreeRosNode::init_timers() {
    RCLCPP_INFO(get_logger(), "Initializing ROS timers...");

    robot_state_timer = this->create_wall_timer(
        100ms, std::bind(&UnitreeRosNode::robot_state_callback, this));

    cmd_vel_reset_timer = this->create_wall_timer(
        1ms, std::bind(&UnitreeRosNode::cmd_vel_reset_callback, this));

    check_robot_battery_timer_ = this->create_wall_timer(
        1min, std::bind(&UnitreeRosNode::check_robot_battery_callback_, this));

    RCLCPP_INFO(get_logger(), "Finished initializing ROS timers!");
}

void UnitreeRosNode::cmd_vel_callback(const geometry_msgs::msg::Twist::UniquePtr msg) {
    unitree_driver.walk_w_vel(msg->linear.x, msg->linear.y, msg->angular.z);
    prev_cmd_vel_sent = clock.now();
}

void UnitreeRosNode::check_robot_battery_callback_() {
    auto batt_level = unitree_driver.get_battery_percentage();
    if (batt_level < low_batt_shutdown_threshold) {
        // Battery is low, Stand down the robot
        RCLCPP_ERROR(this->get_logger(),
                     "Robot battery level is low. Currently at: %hhu%%",
                     batt_level);
        unitree_driver.stop();
        throw std::runtime_error("Robot battery low. Shutting down Driver Node");
    }
    RCLCPP_INFO(this->get_logger(), "Robot battery level is at: %hhu%%", batt_level);
}

void UnitreeRosNode::robot_state_callback() {
    auto now = this->get_clock()->now();
    publish_imu(now);
    publish_bms();
    publish_odom(now);
}

void UnitreeRosNode::cmd_vel_reset_callback() {
    auto delta_t = clock.now() - prev_cmd_vel_sent;
    if (delta_t >= 400ms && delta_t <= 402ms) {
        unitree_driver.walk_w_vel(0, 0, 0);
    }
}

void UnitreeRosNode::stand_up_callback(const std_msgs::msg::Empty::UniquePtr msg) {
    msg.get();  // Just to ignore linter warning
    unitree_driver.stand_up();
}
void UnitreeRosNode::stand_down_callback(const std_msgs::msg::Empty::UniquePtr msg) {
    msg.get();  // Just to ignore linter warning
    unitree_driver.stand_down();
}

void UnitreeRosNode::publish_odom(rclcpp::Time time) {
    odom_t odom = unitree_driver.get_odom();

    tf2::Quaternion q;
    q.setRPY(odom.pose.orientation.x, odom.pose.orientation.y, odom.pose.orientation.z);
    odom.pose.orientation.x = q.getX();
    odom.pose.orientation.y = q.getY();
    odom.pose.orientation.z = q.getZ();
    odom.pose.orientation.w = q.getW();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.child_frame_id = odom_child_frame_id;
    serialize(odom_msg, odom);
    odom_pub->publish(odom_msg);
    publish_odom_tf(time, odom);
}

void UnitreeRosNode::publish_imu(rclcpp::Time time) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = imu_frame_id;
    serialize(imu_msg, unitree_driver.get_imu());
    imu_pub->publish(imu_msg);
}

void UnitreeRosNode::publish_bms() {
    unitree_ros::msg::BmsState bms_msg;
    serialize(bms_msg, unitree_driver.get_bms());
    bms_pub->publish(bms_msg);
}

void UnitreeRosNode::publish_odom_tf(rclcpp::Time time, odom_t odom) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_id;
    transform.child_frame_id = odom_child_frame_id;

    transform.transform.translation.x = odom.pose.position.x;
    transform.transform.translation.y = odom.pose.position.y;
    transform.transform.translation.z = odom.pose.position.z;

    transform.transform.rotation.x = odom.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.orientation.z;
    transform.transform.rotation.w = odom.pose.orientation.w;

    tf_broadcaster->sendTransform(transform);
}
