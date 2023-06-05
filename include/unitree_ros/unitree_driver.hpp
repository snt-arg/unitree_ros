#ifndef UNITREE_DRIVER_H
#define UNITREE_DRIVER_H

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <unitree_ros/unitree_data.hpp>

#include "unitree_legged_sdk/comm.h"

class UnitreeDriver {
    // UDP Connection
   private:
    std::string ip_addr = "192.168.12.1";
    int target_port = 8082;
    int local_port = 8090;
    UNITREE_LEGGED_SDK::UDP udp_connection_;

    UNITREE_LEGGED_SDK::HighCmd high_cmd = {};
    UNITREE_LEGGED_SDK::HighState high_state = {};

    // Robot settings
   private:
    mode_enum curr_mode = mode_enum::MODE_IDDLE;
    gaitype_enum curr_gait_type = gaitype_enum::GAITYPE_IDDLE;
    velocity_t curr_velocity_cmd = {0, 0, 0};
    uint8_t speed_level = speed_level_enum::LOW_SPEED;

   public:
    UnitreeDriver(std::string ip_addr_ = "192.168.12.1", int target_port_ = 8082);
    ~UnitreeDriver();
    /**
     * @brief Stand down the robot
     *
     */
    void stand_up();

    /**
     * @brief Stand up the robot
     *
     */
    void stand_down();
    /**
     * @brief Walk with velocity commands
     *
     * @param x
     * @param y
     * @param yaw
     */
    void walk_w_vel(float x, float y, float yaw);
    /**
     * @brief Walk with position commands
     *
     * @param position
     * @param orientation
     */
    void walk_w_pos(position_t position, orientation_t orientation);
    void damping_mode();
    void illuminate_foot_led(UNITREE_LEGGED_SDK::LED led);
    odom_t get_odom();
    UNITREE_LEGGED_SDK::IMU get_imu();
    UNITREE_LEGGED_SDK::BmsState get_bms();
    sensor_ranges_t get_sensor_ranges();
    void set_mode(mode_enum mode);
    void set_gaitype(gaitype_enum gaitype);
    void stop();

   private:
    void send_high_cmd_();
    void recv_high_state_();
    bool is_connection_established_();
    position_t get_position();
    orientation_t get_orientation();
    velocity_t get_velocity();
};

#endif  // !#ifndef UNITREE_DRIVER_H
