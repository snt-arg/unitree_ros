#ifndef UNITREE_DRIVER_LOWLEVEL_H
#define UNITREE_DRIVER_LOWLEVEL_H

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <unitree_ros/common_defines.hpp>

class UnitreeDriverLowLevel {
 public:
  struct MotorCommand {
    uint8_t mode = 0x00;
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
  };

  struct MotorState {
    uint8_t mode = 0;
    float q = 0.0f;
    float dq = 0.0f;
    float ddq = 0.0f;
    float tauEst = 0.0f;
    float q_raw = 0.0f;
    float dq_raw = 0.0f;
    float ddq_raw = 0.0f;
    int8_t temperature = 0;
  };

  struct ImuState {
    std::array<float, 4> quaternion{};
    std::array<float, 3> gyroscope{};
    std::array<float, 3> accelerometer{};
    std::array<float, 3> rpy{};
    int8_t temperature = 0;
  };

  struct BmsState {
    uint8_t version_h = 0;
    uint8_t version_l = 0;
    uint8_t bms_status = 0;
    uint8_t SOC = 0;
    int32_t current = 0;
    uint16_t cycle = 0;
    std::array<int8_t, 2> BQ_NTC{};
    std::array<int8_t, 2> MCU_NTC{};
    std::array<uint16_t, 10> cell_vol{};
  };

  struct LowState {
    ImuState imu;
    std::array<MotorState, 20> motorState;
    BmsState bms;
    std::array<int16_t, 4> footForce{};
    std::array<int16_t, 4> footForceEst{};
    uint32_t tick = 0;
    std::array<uint8_t, 40> wirelessRemote{};
  };

  struct LowCmd {
    std::array<MotorCommand, 20> motorCmd;
    BmsState bms;
    std::array<uint8_t, 40> wirelessRemote{};
  };

  explicit UnitreeDriverLowLevel(
      std::string ip_addr = "192.168.123.161", int target_port = 8082);
  ~UnitreeDriverLowLevel();

  void send_low_cmd(const LowCmd& cmd);
  LowState get_low_state() const;
  std::array<MotorState, 12> get_leg_joint_states() const;
  void set_damping_mode();
  void set_position_mode(float kp = 60.0f, float kd = 3.0f);
  bool is_connection_established() const;

 private:
  static constexpr int LOCAL_PORT = 8090;
  static constexpr int NUM_LEG_JOINTS = 12;
  static constexpr int NUM_TOTAL_MOTORS = 20;

  std::string ip_addr_;
  int target_port_;
  UNITREE_LEGGED_SDK::UDP udp_connection_;
  UNITREE_LEGGED_SDK::LowCmd sdk_low_cmd_{};
  UNITREE_LEGGED_SDK::LowState sdk_low_state_{};

  std::thread recv_state_thread_;
  std::atomic<bool> recv_state_thread_stop_flag_{false};
  std::atomic<bool> connection_established_{false};

  std::array<int, 12> leg_joint_indices_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  void recv_low_state_loop_();
  bool is_connection_established_();
  void init_cmd_data_();
  uint32_t crc32_core_(const uint32_t* ptr, uint32_t len);
  void compute_crc_(UNITREE_LEGGED_SDK::LowCmd& cmd);
};

#endif