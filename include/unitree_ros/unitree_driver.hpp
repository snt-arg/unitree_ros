#ifndef UNITREE_DRIVER_H
#define UNITREE_DRIVER_H

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <unitree_ros/unitree_data.hpp>

#include "unitree_legged_sdk/comm.h"

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

   public:
    /**
     * @brief Constructor of the class UnitreeDriver.
     *
     * @param ip_addr: Address to be used to communicate with robot (default:
     * 192.168.12.1)
     * @param target_port_: Port to be used to communicate with robot (default: 8082)
     */
    UnitreeDriver(std::string ip_addr = "192.168.123.161", int target_port = 8082);
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
     * @param x: forward/backards direction
     * @param y: left/right direction
     * @param yaw: rotation along the z axis
     */
    void walk_w_vel(float x, float y, float yaw);

    /**
     * @brief Move robot with position commands
     *
     * @param position: {x, y z}
     * @param orientation: {z, y, z, w}
     */
    void walk_w_pos(position_t position, orientation_t orientation);

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
    orientation_t get_orientation_();

    /**
     * @brief Helper method to retrieve robot's velocity from High State
     */
    velocity_t get_velocity_();
};

#endif  // !#ifndef UNITREE_DRIVER_H
