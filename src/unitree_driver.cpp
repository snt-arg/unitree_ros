#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <ostream>
#include <unitree_ros/unitree_driver.hpp>

#include "FaceLightClient.h"
#include "unitree_legged_sdk/comm.h"
#include "unitree_ros/unitree_data.hpp"

UnitreeDriver::UnitreeDriver(std::string ip_addr, int target_port)
    : ip_addr_(ip_addr),
      target_port_(target_port),
      udp_connection_(UNITREE_LEGGED_SDK::HIGHLEVEL,
                      local_port_,
                      ip_addr.c_str(),
                      target_port),
      light_client_(),
      face_led_thread_(&UnitreeDriver::show_robot_status, this),
      face_led_thread_stop_flag_(false) {
    // Check if the connection is established
    if (!is_connection_established_()) {
        throw std::runtime_error(
            "Connection to the robot could not be established, shutting down.");
    }

    // Initialize the high level command and state
    udp_connection_.InitCmdData(high_cmd_);
    set_head_led(255, 255, 255);

    stand_up();
}

UnitreeDriver::~UnitreeDriver() {
    face_led_thread_stop_flag_ = true;
    face_led_thread_.join();
    stop();
}

// -----------------------------------------------------------------------------
// -                                 Getters                                   -
// -----------------------------------------------------------------------------

odom_t UnitreeDriver::get_odom() {
    recv_high_state_();
    position_t position = get_position_();
    orientation_t orientation = get_orientation_();
    velocity_t velocity = get_velocity_();
    pose_t pose = {position, orientation};
    return {pose, velocity};
}

sensor_ranges_t UnitreeDriver::get_radar_ranges() {
    recv_high_state_();
    sensor_ranges_t ranges;
    ranges.front = high_state_.rangeObstacle[0];
    ranges.left = high_state_.rangeObstacle[1];
    ranges.right = high_state_.rangeObstacle[2];
    return ranges;
}

UNITREE_LEGGED_SDK::IMU UnitreeDriver::get_imu() {
    recv_high_state_();
    return high_state_.imu;
}

UNITREE_LEGGED_SDK::BmsState UnitreeDriver::get_bms() {
    recv_high_state_();
    return high_state_.bms;
}

uint8_t UnitreeDriver::get_battery_percentage() {
    recv_high_state_();
    return high_state_.bms.SOC;
}

// -----------------------------------------------------------------------------
// -                                  Setters                                  -
// -----------------------------------------------------------------------------

void UnitreeDriver::set_mode(mode_enum mode) { curr_mode_ = mode; }

void UnitreeDriver::set_gaitype(gaitype_enum gait_type) { curr_gait_type_ = gait_type; }

void UnitreeDriver::enable_obstacle_avoidance(bool flag) {
    use_obstacle_avoidance_ = flag;
}

// -----------------------------------------------------------------------------
// -                             Robot Functions                               -
// -----------------------------------------------------------------------------

void UnitreeDriver::stand_down() {
    std::cout << "STANDING DOWN" << std::endl;
    walk_w_vel(0, 0, 0);
    set_gaitype(gaitype_enum::GAITYPE_IDDLE);
    set_mode(mode_enum::STAND_DOWN);
    send_high_cmd_();
    robot_status = robot_status_e::IDDLE;
    set_head_led(255, 255, 255);
}

void UnitreeDriver::stand_up() {
    std::cout << "STANDING UP" << std::endl;
    if (use_obstacle_avoidance_)
        set_gaitype(gaitype_enum::TROT_OBSTACLE);
    else
        set_gaitype(gaitype_enum::TROT);
    set_mode(mode_enum::STAND_UP);
    send_high_cmd_();
    robot_status = robot_status_e::READY;
    set_head_led(0, 255, 0);
}

void UnitreeDriver::walk_w_vel(float x, float y, float yaw) {
    robot_status = robot_status_e::MOVING;
    if (use_obstacle_avoidance_)
        set_gaitype(gaitype_enum::TROT_OBSTACLE);
    else
        set_gaitype(gaitype_enum::TROT);
    set_mode(mode_enum::WALK_W_VEL);
    curr_velocity_cmd_ = {x, y, yaw};
    send_high_cmd_();
}

void UnitreeDriver::walk_w_pos(position_t position, orientation_t orientation) {
    robot_status = robot_status_e::MOVING;
    if (use_obstacle_avoidance_)
        set_gaitype(gaitype_enum::TROT_OBSTACLE);
    else
        set_gaitype(gaitype_enum::TROT);
    set_mode(mode_enum::STAND_UP);
    high_cmd_.position[0] = position.x;
    high_cmd_.position[1] = position.y;
    high_cmd_.euler[0] = orientation.x;
    high_cmd_.euler[1] = orientation.y;
    high_cmd_.euler[2] = orientation.z;
    send_high_cmd_();
}

void UnitreeDriver::damping_mode() {
    recv_high_state_();
    if (high_state_.mode == mode_enum::STAND_DOWN) {
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
    sleep(3);  // Giving some delay so that the robot has the time to stand down
    std::cout << "Disabling robot motors" << std::endl;
    damping_mode();
}

void UnitreeDriver::set_head_led(UNITREE_LEGGED_SDK::LED led) {
    const uint8_t color[3] = {led.r, led.g, led.b};
    light_client_.setAllLed(color);
    light_client_.sendCmd();
}

void UnitreeDriver::set_head_led(uint8_t r, uint8_t g, uint8_t b) {
    const uint8_t color[3] = {r, g, b};
    light_client_.setAllLed(color);
    light_client_.sendCmd();
}

// -----------------------------------------------------------------------------
// -                             Private Functions                             -
// -----------------------------------------------------------------------------

position_t UnitreeDriver::get_position_() {
    return {high_state_.position[0], high_state_.position[1], high_state_.position[2]};
}

orientation_t UnitreeDriver::get_orientation_() {
    return {high_state_.imu.rpy[0], high_state_.imu.rpy[1], high_state_.imu.rpy[2], 0};
}

velocity_t UnitreeDriver::get_velocity_() {
    return {high_state_.velocity[0], high_state_.velocity[1], high_state_.yawSpeed};
}

bool UnitreeDriver::is_connection_established_() {
    std::cout << "Checking if robot connection is availble" << std::endl;
    string cmd = "ping -c1 -s1 ";
    cmd = cmd + ip_addr_ + " > /dev/null 2>&1";
    int exit_status = std::system(cmd.c_str());

    if (exit_status == 0) {
        std::cout << "Connection is available" << std::endl;
        return true;
    }
    std::cout << "CONNECTION IS NOT AVAILABLE" << std::endl;
    return false;
}

void UnitreeDriver::blink_face_led(uint8_t r, uint8_t g, uint8_t b) {
    set_head_led(r, g, b);
    sleep(1);
    set_head_led(0, 0, 0);
    sleep(1);
}

void UnitreeDriver::show_robot_status() {
    while (!face_led_thread_stop_flag_) {
        switch (robot_status) {
            case robot_status_e::READY:
                blink_face_led(0, 255, 0);
                break;
            case robot_status_e::IDDLE:
                blink_face_led(255, 255, 255);
                break;
            case robot_status_e::MOVING:
                blink_face_led(0, 170, 255);
                break;
            case robot_status_e::BATTERY_LOW:
                blink_face_led(255, 200, 0);
                break;
            case robot_status_e::ERROR:
                blink_face_led(255, 0, 0);
                break;
        }
    }
}

void UnitreeDriver::send_high_cmd_() {
    high_cmd_.mode = curr_mode_;
    high_cmd_.gaitType = curr_gait_type_;
    high_cmd_.speedLevel = speed_level_;

    high_cmd_.velocity[0] = curr_velocity_cmd_.x;
    high_cmd_.velocity[1] = curr_velocity_cmd_.y;
    high_cmd_.yawSpeed = curr_velocity_cmd_.yaw;

    udp_connection_.SetSend(high_cmd_);
    udp_connection_.Send();
}

void UnitreeDriver::recv_high_state_() {
    udp_connection_.Send();
    udp_connection_.Recv();
    udp_connection_.GetRecv(high_state_);
}
