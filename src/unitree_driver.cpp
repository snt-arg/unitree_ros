#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <ostream>
#include <unitree_ros/unitree_driver.hpp>

#include "unitree_ros/unitree_data.hpp"

UnitreeDriver::UnitreeDriver(std::string ip_addr_, int target_port_)
    : udp_connection_(UNITREE_LEGGED_SDK::HIGHLEVEL,
                      local_port,
                      ip_addr_.c_str(),
                      target_port_) {
    // Check if the connection is established
    if (!is_connection_established_()) {
        throw std::runtime_error(
            "Connection to the robot could not be established, shutting down.");
    }

    // Initialize the high level command and state
    udp_connection_.InitCmdData(high_cmd);

    stand_up();
}

UnitreeDriver::~UnitreeDriver() { stop(); }

// -----------------------------------------------------------------------------
// -                                 Getters                                   -
// -----------------------------------------------------------------------------

position_t UnitreeDriver::get_position() {
    return {high_state.position[0], high_state.position[1], high_state.position[2]};
}

orientation_t UnitreeDriver::get_orientation() {
    return {high_state.imu.rpy[0], high_state.imu.rpy[1], high_state.imu.rpy[2], 0};
}

velocity_t UnitreeDriver::get_velocity() {
    return {high_state.velocity[0], high_state.velocity[1], high_state.yawSpeed};
}

odom_t UnitreeDriver::get_odom() {
    recv_high_state_();
    position_t position = get_position();
    orientation_t orientation = get_orientation();
    velocity_t velocity = get_velocity();
    pose_t pose = {position, orientation};
    return {pose, velocity};
}

sensor_ranges_t UnitreeDriver::get_sensor_ranges() {
    recv_high_state_();
    sensor_ranges_t ranges;
    ranges.front = high_state.rangeObstacle[0];
    ranges.left = high_state.rangeObstacle[1];
    ranges.right = high_state.rangeObstacle[2];
    return ranges;
}

UNITREE_LEGGED_SDK::IMU UnitreeDriver::get_imu() {
    recv_high_state_();
    return high_state.imu;
}

UNITREE_LEGGED_SDK::BmsState UnitreeDriver::get_bms() {
    recv_high_state_();
    return high_state.bms;
}

uint8_t UnitreeDriver::get_battery_percentage() {
    recv_high_state_();
    return high_state.bms.SOC;
}

// -----------------------------------------------------------------------------
// -                                  Setters                                  -
// -----------------------------------------------------------------------------

void UnitreeDriver::set_mode(mode_enum mode) { curr_mode = mode; }

void UnitreeDriver::set_gaitype(gaitype_enum gait_type) { curr_gait_type = gait_type; }

void UnitreeDriver::enable_obstacle_avoidance(bool flag) {
    use_obstacle_avoidance = flag;
}

// -----------------------------------------------------------------------------
// -                             Robot Functions                               -
// -----------------------------------------------------------------------------

void UnitreeDriver::stand_down() {
    std::cout << "STANDIND DOWN" << std::endl;
    walk_w_vel(0, 0, 0);
    set_gaitype(gaitype_enum::GAITYPE_IDDLE);
    set_mode(mode_enum::STAND_DOWN);
    send_high_cmd_();
}

void UnitreeDriver::stand_up() {
    std::cout << "STANDIND UP" << std::endl;
    if (use_obstacle_avoidance)
        set_gaitype(gaitype_enum::TROT_OBSTACLE);
    else
        set_gaitype(gaitype_enum::TROT);
    send_high_cmd_();
    set_mode(mode_enum::STAND_UP);
    send_high_cmd_();
}

void UnitreeDriver::walk_w_vel(float x, float y, float yaw) {
    if (use_obstacle_avoidance)
        set_gaitype(gaitype_enum::TROT_OBSTACLE);
    else
        set_gaitype(gaitype_enum::TROT);
    set_mode(mode_enum::WALK_W_VEL);
    curr_velocity_cmd = {x, y, yaw};
    send_high_cmd_();
}

void UnitreeDriver::walk_w_pos(position_t position, orientation_t orientation) {
    high_cmd.mode = mode_enum::WALK_W_POS;
    high_cmd.gaitType = curr_gait_type;
    high_cmd.position[0] = position.x;
    high_cmd.position[1] = position.y;
    high_cmd.euler[0] = orientation.x;
    high_cmd.euler[1] = orientation.y;
    high_cmd.euler[2] = orientation.z;
    send_high_cmd_();
}

void UnitreeDriver::damping_mode() {
    recv_high_state_();
    if (high_state.mode == mode_enum::STAND_DOWN) {
        set_mode(mode_enum::DAMPING_MODE);
        send_high_cmd_();
    } else {
        std::cout << "Robot is not in STAND_DOWN mode. Make sure to stand down the "
                     "robot first"
                  << std::endl;
    }
}

void UnitreeDriver::stop() {
    stand_down();
    std::cout << "Waiting for robot to stand down" << std::endl;
    sleep(3);
    std::cout << "Disabling robot motors" << std::endl;
    damping_mode();
}

// -----------------------------------------------------------------------------
// -                             Private Functions                             -
// -----------------------------------------------------------------------------

bool UnitreeDriver::is_connection_established_() {
    std::cout << "Checking if robot connection is availble" << std::endl;
    string cmd = "ping -c1 -s1 ";
    cmd = cmd + ip_addr + " > /dev/null 2>&1";
    int exit_status = std::system(cmd.c_str());

    if (exit_status == 0) {
        std::cout << "Connection is available" << std::endl;
        return true;
    }
    std::cout << "Connection is not available" << std::endl;
    return false;
}

void UnitreeDriver::send_high_cmd_() {
    high_cmd.mode = curr_mode;
    high_cmd.gaitType = curr_gait_type;
    high_cmd.speedLevel = speed_level;

    high_cmd.velocity[0] = curr_velocity_cmd.x;
    high_cmd.velocity[1] = curr_velocity_cmd.y;
    high_cmd.yawSpeed = curr_velocity_cmd.yaw;

    udp_connection_.SetSend(high_cmd);
    udp_connection_.Send();
}

void UnitreeDriver::recv_high_state_() {
    udp_connection_.Send();
    udp_connection_.Recv();
    udp_connection_.GetRecv(high_state);
}
