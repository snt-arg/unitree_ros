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

#ifndef UNITREE_DRIVER_H
#define UNITREE_DRIVER_H

#include <FaceLightClient.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <atomic>
#include <memory>
#include <thread>
#include <unitree_ros/common_defines.hpp>

#include "unitree_legged_sdk/comm.h"

/*
 * @brief Class Unitree driver which is use for the main interaction with
 * the Unitree Go1 robot.
 */
class UnitreeDriver {
    // Unitree SDK related
   private:
    std::string ip_addr_ = "192.168.123.161";
    int target_port_ = 8082;
    int local_port_ = 8090;
    UNITREE_LEGGED_SDK::UDP udp_connection_;

    UNITREE_LEGGED_SDK::HighCmd high_cmd_ = {};
    UNITREE_LEGGED_SDK::HighState high_state_ = {};

    // Robot settings
    mode_enum curr_mode_ = mode_enum::MODE_IDDLE;
    gaitype_enum curr_gait_type_ = gaitype_enum::GAITYPE_IDDLE;
    velocity_t curr_velocity_cmd_ = {0, 0, 0};
    uint8_t speed_level_ = speed_level_enum::LOW_SPEED;
    bool use_obstacle_avoidance_ = false;

    // Faceled
    FaceLightClient light_client_;

    // Robot status
    robot_status_e robot_status = robot_status_e::IDDLE;

    // Threads
    std::thread face_led_thread_;
    std::atomic<bool> face_led_thread_stop_flag_;
    std::thread recv_state_thread_;
    std::atomic<bool> recv_state_thread_stop_flag_;

   public:
    /**
     * @brief Constructor of the class UnitreeDriver.
     *
     * @param ip_addr: Address to be used to communicate with robot (default:
     * 192.168.12.1)
     * @param target_port_: Port to be used to communicate with robot (default: 8082)
     */
    UnitreeDriver(std::string ip_addr = "192.168.123.161", int target_port = 8082);
    /*
     * @brief Deconstructor for the class UnitreeDriver. When called, it makes
     * the robot stand down and deactivate the motors.
     */
    ~UnitreeDriver();

    /**
     * @brief Stand up the robot.
     *
     */
    void stand_up();

    /**
     * @brief Stand down the robot.
     *
     */
    void stand_down();

    /**
     * @brief Go into damping mode. Motor will be disabled so use with caution.
     * Make the robot stand down before using this method.
     *
     * @comment In case the robot is not in stand mode, this will not be executed.
     */
    void damping_mode();

    /**
     * @brief Move robot with with velocity commands
     *
     * @param x: forward(+)/backards(-) direction
     * @param y: left(+)/right(-) direction
     * @param yaw: rotation along the z axis (+ccw)
     */
    void walk_w_vel(float x, float y, float yaw);

    /**
     * @brief Move robot with position commands relative where the robot was booted
     * (Never tested)
     *
     * @param position: {x, y z}
     * @param orientation: {z, y, z, w}
     */
    void walk_w_pos(position_t position, quarternion_t orientation);

    /**
     * @brief Retrieves odometry from the robot
     *
     * @return odom_t: The odometry from the robot
     */
    odom_t get_odom();

    /**
     * @brief Retrieves IMU data from the robot
     *
     * @return UNITREE_LEGGED_SDK::IMU: The IMU data from the robot
     */
    UNITREE_LEGGED_SDK::IMU get_imu();

    /**
     * @brief Retrieves BMS (Battery Management System) data from the robot
     * it
     *
     * @return UNITREE_LEGGED_SDK::BMS: The BMS data from the robot
     */
    UNITREE_LEGGED_SDK::BmsState get_bms();

    /**
     * @brief Retrieves the distances comming from the radar sensors from the front and
     * sides of the robot
     *
     * @return sensor_ranges_t: the 3 ranges from the radars
     */
    sensor_ranges_t get_radar_ranges();

    /**
     * @brief Retrieves the current battery percentage of the robot's battery
     *
     * @return uint8_t: battery percentage (0-100%)
     */
    uint8_t get_battery_percentage();

    /**
     * @brief Changes the current mode the robot should be in
     * The possible modes are:
     *   - MODE_IDDLE = 0,
     *   - WALK_W_VEL = 2,
     *   - WALK_W_POS = 3,
     *   - STAND_DOWN = 5,
     *   - STAND_UP = 6,
     *   - DAMPING_MODE = 7,
     *
     * @param mode: mode to be set
     */
    void set_mode(mode_enum mode);

    /**
     * @brief Changes the current gaitype the robot should be in
     * The possible modes are:
     *   - GAITYPE_IDDLE = 0,
     *   - TROT = 1,
     *   - TROT_RUNNING = 2,
     *   - CLIMB_STAIR = 3,
     *   - TROT_OBSTACLE = 4,
     *
     * @param gaitype: gaitype to be set
     */
    void set_gaitype(gaitype_enum gaitype);

    /**
     * @brief Enables/disables robot's obstacle avoidance
     *
     * @param flag: true: enables | false: disables
     */
    void enable_obstacle_avoidance(bool flag);

    /**
     * @brief Makes the robot stand down and disable motors.
     */
    void stop();

    /**
     * @brief Sets the robot face LEDs color
     *
     * @param r: percentage of the red color (0-254)
     * @param g: percentage of the green color (0-254)
     * @param b: percentage of the blue color (0-254)
     */
    void set_head_led(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Sets the robot face LEDs color
     *
     * @param led: struct containing the 3 primary colors
     */
    void set_head_led(UNITREE_LEGGED_SDK::LED led);

    /**
     * @brief Sends the colors to the face LEDs based on the robot status.
     */
    void update_robot_status();

   private:
    /**
     * @brief Sends the current High level command to the robot.
     */
    void send_high_cmd_();

    /**
     * @brief Recieves the current High State from the robot and updates the high_state
     * attribute of the driver
     */
    void recv_high_state_();

    /**
     * @brief Checks if there is a connection available with the robot.
     *
     * @returns true if connection available, false otherwise
     */
    bool is_connection_established_();

    /**
     * @brief Helper method to retrieve robot's position from High State
     */
    position_t get_position_();

    /**
     * @brief Helper method to retrieve robot's orientation from High State
     */
    quarternion_t get_quaternion_();

    /**
     * @brief Helper method to retrieve robot's velocity from High State
     */
    velocity_t get_velocity_();

    /**
     * Makes the robot's face LEDs blink.
     *
     * @param r: percentage of the red color (0-254)
     * @param g: percentage of the green color (0-254)
     * @param b: percentage of the blue color (0-254)
     */
    void blink_face_led(uint8_t r, uint8_t g, uint8_t b);
    /**
     * Makes the robot's face LEDs blink.
     *
     * @param led: struct containing the 3 primary colors
     */
    void blink_face_led(UNITREE_LEGGED_SDK::LED led);
};

#endif  // !#ifndef UNITREE_DRIVER_H
