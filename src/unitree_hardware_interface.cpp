#include <unitree_ros/unitree_hardware_interface.hpp>
#include <rclcpp/logging.hpp>

namespace unitree_ros
{

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Initializing Unitree Hardware Interface...");

  for (const auto& param : info.hardware_parameters) {
    if (param.first == "robot_ip") {
      robot_ip_ = param.second;
    } else if (param.first == "robot_port") {
      robot_port_ = std::stoi(param.second);
    } else if (param.first == "default_kp") {
      default_kp_ = std::stof(param.second);
    } else if (param.first == "default_kd") {
      default_kd_ = std::stof(param.second);
    }
  }

  hw_positions_.resize(joint_names_.size(), 0.0);
  hw_velocities_.resize(joint_names_.size(), 0.0);
  hw_efforts_.resize(joint_names_.size(), 0.0);
  hw_position_commands_.resize(joint_names_.size(), 0.0);
  hw_velocity_commands_.resize(joint_names_.size(), 0.0);
  hw_effort_commands_.resize(joint_names_.size(), 0.0);

  try {
    driver_ = std::make_unique<UnitreeDriverLowLevel>(robot_ip_, robot_port_);
    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Driver connected successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"), "Failed to connect: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_->set_position_mode(default_kp_, default_kd_);

  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Hardware interface initialized!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UnitreeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnitreeHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Activating hardware interface...");
  
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hw_position_commands_[i] = hw_positions_[i];
    hw_velocity_commands_[i] = 0.0;
    hw_effort_commands_[i] = 0.0;
  }

  driver_->set_position_mode(default_kp_, default_kd_);
  
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Hardware interface activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Deactivating hardware interface...");
  
  driver_->set_damping_mode();
  
  RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "Hardware interface deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeHardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  auto leg_states = driver_->get_leg_joint_states();
  
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    int sdk_idx = joint_to_sdk_index_[i];
    hw_positions_[i] = leg_states[sdk_idx].q;
    hw_velocities_[i] = leg_states[sdk_idx].dq;
    hw_efforts_[i] = leg_states[sdk_idx].tauEst;
  }

  if (first_read_) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      hw_position_commands_[i] = hw_positions_[i];
    }
    first_read_ = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeHardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  UnitreeDriverLowLevel::LowCmd cmd{};
  
  for (int i = 0; i < 20; ++i) {
    cmd.motorCmd[i].mode = 0x00;
    cmd.motorCmd[i].q = 0.0f;
    cmd.motorCmd[i].dq = 0.0f;
    cmd.motorCmd[i].tau = 0.0f;
    cmd.motorCmd[i].kp = 0.0f;
    cmd.motorCmd[i].kd = 0.0f;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    int sdk_idx = joint_to_sdk_index_[i];
    cmd.motorCmd[sdk_idx].mode = 0x0A;
    cmd.motorCmd[sdk_idx].q = static_cast<float>(hw_position_commands_[i]);
    cmd.motorCmd[sdk_idx].dq = static_cast<float>(hw_velocity_commands_[i]);
    cmd.motorCmd[sdk_idx].tau = static_cast<float>(hw_effort_commands_[i]);
    cmd.motorCmd[sdk_idx].kp = default_kp_;
    cmd.motorCmd[sdk_idx].kd = default_kd_;
  }

  driver_->send_low_cmd(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace unitree_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(unitree_ros::UnitreeHardwareInterface, hardware_interface::SystemInterface)