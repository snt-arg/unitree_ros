#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <unitree_ros/unitree_driver_lowlevel.hpp>

UnitreeDriverLowLevel::UnitreeDriverLowLevel(std::string ip_addr, int target_port)
    : ip_addr_(std::move(ip_addr)),
      target_port_(target_port),
      udp_connection_(UNITREE_LEGGED_SDK::LOWLEVEL, LOCAL_PORT, ip_addr_.c_str(), target_port_) {
  std::cout << "Initializing Unitree LowLevel Driver..." << std::endl;
  std::cout << "IP: " << ip_addr_ << ", Target Port: " << target_port_ << ", Local Port: " << LOCAL_PORT << std::endl;

  sleep(2);

  if (!is_connection_established_()) {
    throw std::runtime_error("Connection to the robot could not be established, shutting down.");
  }

  init_cmd_data_();
  connection_established_ = true;
  std::cout << "LowLevel connection established!" << std::endl;

  recv_state_thread_ = std::thread(&UnitreeDriverLowLevel::recv_low_state_loop_, this);
  std::cout << "LowLevel state receiver thread started" << std::endl;
}

UnitreeDriverLowLevel::~UnitreeDriverLowLevel() {
  recv_state_thread_stop_flag_ = true;
  if (recv_state_thread_.joinable()) {
    recv_state_thread_.join();
  }
  std::cout << "LowLevel driver shut down" << std::endl;
}

void UnitreeDriverLowLevel::init_cmd_data_() {
  udp_connection_.InitCmdData(sdk_low_cmd_);

  for (int i = 0; i < NUM_TOTAL_MOTORS; ++i) {
    sdk_low_cmd_.motorCmd[i].mode = 0x00;
    sdk_low_cmd_.motorCmd[i].q = 0.0f;
    sdk_low_cmd_.motorCmd[i].dq = 0.0f;
    sdk_low_cmd_.motorCmd[i].tau = 0.0f;
    sdk_low_cmd_.motorCmd[i].Kp = 0.0f;
    sdk_low_cmd_.motorCmd[i].Kd = 0.0f;
  }
}

void UnitreeDriverLowLevel::recv_low_state_loop_() {
  while (!recv_state_thread_stop_flag_) {
    udp_connection_.Recv();
    udp_connection_.GetRecv(sdk_low_state_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool UnitreeDriverLowLevel::is_connection_established_() {
  std::cout << "Checking if robot connection is available..." << std::endl;
  std::string cmd = "ping -c1 -s1 " + ip_addr_ + " > /dev/null 2>&1";
  int exit_status = std::system(cmd.c_str());

  if (exit_status == 0) {
    std::cout << "Connection is available" << std::endl;
    return true;
  }
  std::cout << "CONNECTION IS NOT AVAILABLE" << std::endl;
  return false;
}

bool UnitreeDriverLowLevel::is_connection_established() const {
  return connection_established_;
}

void UnitreeDriverLowLevel::send_low_cmd(const LowCmd& cmd) {
  for (int i = 0; i < NUM_TOTAL_MOTORS; ++i) {
    sdk_low_cmd_.motorCmd[i].mode = cmd.motorCmd[i].mode;
    sdk_low_cmd_.motorCmd[i].q = cmd.motorCmd[i].q;
    sdk_low_cmd_.motorCmd[i].dq = cmd.motorCmd[i].dq;
    sdk_low_cmd_.motorCmd[i].tau = cmd.motorCmd[i].tau;
    sdk_low_cmd_.motorCmd[i].Kp = cmd.motorCmd[i].kp;
    sdk_low_cmd_.motorCmd[i].Kd = cmd.motorCmd[i].kd;
  }

  compute_crc_(sdk_low_cmd_);
  udp_connection_.SetSend(sdk_low_cmd_);
  udp_connection_.Send();
}

UnitreeDriverLowLevel::LowState UnitreeDriverLowLevel::get_low_state() const {
  LowState state;
  state.imu.quaternion = sdk_low_state_.imu.quaternion;
  state.imu.gyroscope = sdk_low_state_.imu.gyroscope;
  state.imu.accelerometer = sdk_low_state_.imu.accelerometer;
  state.imu.rpy = sdk_low_state_.imu.rpy;
  state.imu.temperature = sdk_low_state_.imu.temperature;

  for (int i = 0; i < NUM_TOTAL_MOTORS; ++i) {
    state.motorState[i].mode = sdk_low_state_.motorState[i].mode;
    state.motorState[i].q = sdk_low_state_.motorState[i].q;
    state.motorState[i].dq = sdk_low_state_.motorState[i].dq;
    state.motorState[i].ddq = sdk_low_state_.motorState[i].ddq;
    state.motorState[i].tauEst = sdk_low_state_.motorState[i].tauEst;
    state.motorState[i].q_raw = sdk_low_state_.motorState[i].q_raw;
    state.motorState[i].dq_raw = sdk_low_state_.motorState[i].dq_raw;
    state.motorState[i].ddq_raw = sdk_low_state_.motorState[i].ddq_raw;
    state.motorState[i].temperature = sdk_low_state_.motorState[i].temperature;
  }

  state.bms.version_h = sdk_low_state_.bms.version_h;
  state.bms.version_l = sdk_low_state_.bms.version_l;
  state.bms.bms_status = sdk_low_state_.bms.bms_status;
  state.bms.SOC = sdk_low_state_.bms.SOC;
  state.bms.current = sdk_low_state_.bms.current;
  state.bms.cycle = sdk_low_state_.bms.cycle;
  state.bms.BQ_NTC = sdk_low_state_.bms.BQ_NTC;
  state.bms.MCU_NTC = sdk_low_state_.bms.MCU_NTC;
  state.bms.cell_vol = sdk_low_state_.bms.cell_vol;

  state.footForce = sdk_low_state_.footForce;
  state.footForceEst = sdk_low_state_.footForceEst;
  state.tick = sdk_low_state_.tick;
  state.wirelessRemote = sdk_low_state_.wirelessRemote;

  return state;
}

std::array<UnitreeDriverLowLevel::MotorState, 12> UnitreeDriverLowLevel::get_leg_joint_states() const {
  std::array<MotorState, 12> joint_states;
  for (int i = 0; i < NUM_LEG_JOINTS; ++i) {
    int idx = leg_joint_indices_[i];
    joint_states[i].mode = sdk_low_state_.motorState[idx].mode;
    joint_states[i].q = sdk_low_state_.motorState[idx].q;
    joint_states[i].dq = sdk_low_state_.motorState[idx].dq;
    joint_states[i].ddq = sdk_low_state_.motorState[idx].ddq;
    joint_states[i].tauEst = sdk_low_state_.motorState[idx].tauEst;
    joint_states[i].q_raw = sdk_low_state_.motorState[idx].q_raw;
    joint_states[i].dq_raw = sdk_low_state_.motorState[idx].dq_raw;
    joint_states[i].ddq_raw = sdk_low_state_.motorState[idx].ddq_raw;
    joint_states[i].temperature = sdk_low_state_.motorState[idx].temperature;
  }
  return joint_states;
}

void UnitreeDriverLowLevel::set_damping_mode() {
  for (int i = 0; i < NUM_TOTAL_MOTORS; ++i) {
    sdk_low_cmd_.motorCmd[i].mode = 0x00;
    sdk_low_cmd_.motorCmd[i].q = 0.0f;
    sdk_low_cmd_.motorCmd[i].dq = 0.0f;
    sdk_low_cmd_.motorCmd[i].tau = 0.0f;
    sdk_low_cmd_.motorCmd[i].Kp = 0.0f;
    sdk_low_cmd_.motorCmd[i].Kd = 5.0f;
  }
  compute_crc_(sdk_low_cmd_);
  udp_connection_.SetSend(sdk_low_cmd_);
  udp_connection_.Send();
}

void UnitreeDriverLowLevel::set_position_mode(float kp, float kd) {
  for (int i = 0; i < NUM_TOTAL_MOTORS; ++i) {
    sdk_low_cmd_.motorCmd[i].mode = 0x0A;
    sdk_low_cmd_.motorCmd[i].Kp = kp;
    sdk_low_cmd_.motorCmd[i].Kd = kd;
  }
}

uint32_t UnitreeDriverLowLevel::crc32_core_(const uint32_t* ptr, uint32_t len) {
  static const uint32_t crc_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
};

  uint32_t crc = 0xFFFFFFFF;
  const uint8_t* data = reinterpret_cast<const uint8_t*>(ptr);
  for (uint32_t i = 0; i < len * 4; ++i) {
    crc = crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFF;
}

void UnitreeDriverLowLevel::compute_crc_(UNITREE_LEGGED_SDK::LowCmd& cmd) {
  cmd.crc = 0;
  uint32_t* data = reinterpret_cast<uint32_t*>(&cmd);
  int len = (sizeof(UNITREE_LEGGED_SDK::LowCmd) - 4) / 4;
  cmd.crc = crc32_core_(data, len);
}