#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_ros/msg/low_state.hpp>
#include <unitree_ros/msg/low_cmd.hpp>
#include <unitree_ros/msg/motor_cmd.hpp>
#include <unitree_ros/msg/bms_state.hpp>
#include <unitree_ros/msg/imu_state.hpp>
#include <unitree_ros/unitree_driver_lowlevel.hpp>
#include <unitree_ros/serializers.hpp>

class UnitreeLowLevelNode : public rclcpp::Node
{
public:
  UnitreeLowLevelNode() : Node("unitree_lowlevel_node")
  {
    declare_parameter<std::string>("robot_ip", "192.168.123.161");
    declare_parameter<int>("robot_port", 8082);
    declare_parameter<double>("publish_rate", 500.0);
    declare_parameter<std::string>("joint_states_topic", "joint_states");
    declare_parameter<std::string>("low_state_topic", "low_state");
    declare_parameter<std::string>("low_cmd_topic", "low_cmd");

    get_parameter("robot_ip", robot_ip_);
    get_parameter("robot_port", robot_port_);
    get_parameter("publish_rate", publish_rate_);
    get_parameter("joint_states_topic", joint_states_topic_);
    get_parameter("low_state_topic", low_state_topic_);
    get_parameter("low_cmd_topic", low_cmd_topic_);

    RCLCPP_INFO(get_logger(), "Initializing low-level driver...");
    
    try {
      driver_ = std::make_unique<UnitreeDriverLowLevel>(robot_ip_, robot_port_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize driver: %s", e.what());
      return;
    }

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(1).reliable());
    
    low_state_pub_ = create_publisher<unitree_ros::msg::LowState>(
      low_state_topic_, rclcpp::QoS(1).reliable());

    low_cmd_sub_ = create_subscription<unitree_ros::msg::LowCmd>(
      low_cmd_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&UnitreeLowLevelNode::low_cmd_callback_, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
      std::bind(&UnitreeLowLevelNode::publish_state_, this));

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Low-level node started at %.1f Hz", publish_rate_);
  }

  bool is_initialized() const { return initialized_; }

private:
  void low_cmd_callback_(const unitree_ros::msg::LowCmd::SharedPtr msg)
  {
    UnitreeDriverLowLevel::LowCmd cmd{};
    size_t n = std::min<size_t>(msg->motor_cmd.size(), 20);
    for (size_t i = 0; i < n; ++i) {
      cmd.motorCmd[i].mode = msg->motor_cmd[i].mode;
      cmd.motorCmd[i].q = msg->motor_cmd[i].q;
      cmd.motorCmd[i].dq = msg->motor_cmd[i].dq;
      cmd.motorCmd[i].tau = msg->motor_cmd[i].tau;
      cmd.motorCmd[i].kp = msg->motor_cmd[i].kp;
      cmd.motorCmd[i].kd = msg->motor_cmd[i].kd;
    }
    driver_->send_low_cmd(cmd);
  }

  void publish_state_()
  {
    auto now = this->now();
    
    auto leg_states = driver_->get_leg_joint_states();
    
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = now;
    joint_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    joint_msg.position.resize(12);
    joint_msg.velocity.resize(12);
    joint_msg.effort.resize(12);
    
    for (size_t i = 0; i < 12; ++i) {
      joint_msg.position[i] = leg_states[i].q;
      joint_msg.velocity[i] = leg_states[i].dq;
      joint_msg.effort[i] = leg_states[i].tauEst;
    }
    joint_states_pub_->publish(joint_msg);

    auto low_state = driver_->get_low_state();
    unitree_ros::msg::LowState low_state_msg;
    low_state_msg.stamp = now;
    serialize(low_state_msg, low_state);
    low_state_pub_->publish(low_state_msg);
  }

  std::string robot_ip_;
  int robot_port_;
  double publish_rate_;
  std::string joint_states_topic_;
  std::string low_state_topic_;
  std::string low_cmd_topic_;

  std::unique_ptr<UnitreeDriverLowLevel> driver_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<unitree_ros::msg::LowState>::SharedPtr low_state_pub_;
  rclcpp::Subscription<unitree_ros::msg::LowCmd>::SharedPtr low_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnitreeLowLevelNode>();
  if (!node->is_initialized()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}