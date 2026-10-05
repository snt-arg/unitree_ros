#ifndef UNITREE_HARDWARE_INTERFACE_HPP
#define UNITREE_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <unitree_ros/unitree_driver_lowlevel.hpp>

#include <array>
#include <string>
#include <vector>
#include <memory>

namespace unitree_ros
{

class UnitreeHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unique_ptr<UnitreeDriverLowLevel> driver_;
  std::string robot_ip_ = "192.168.123.161";
  int robot_port_ = 8082;
  float default_kp_ = 60.0f;
  float default_kd_ = 3.0f;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_effort_commands_;

  std::vector<std::string> joint_names_ = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
  };

  const std::array<int, 12> joint_to_sdk_index_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  bool first_read_ = true;
};

}  // namespace unitree_ros

#endif  // UNITREE_HARDWARE_INTERFACE_HPP