# Unitree Go1 Low-Level Control Guide

This document describes how to use the low-level joint control capabilities for the Unitree Go1 quadruped robot in ROS2 (Jazzy/Kilted).

## ⚠️ Safety First

**READ THIS BEFORE RUNNING ANY LOW-LEVEL CONTROL:**

1. **Robot must be suspended/harnessed** - The robot MUST be off the ground (hanging in a harness) before enabling low-level control
2. **Pre-condition sequence required:**
   - Press `L2 + A` on the remote to make the robot sit down
   - Press `L1 + L2 + Start` to enter low-level control mode
   - Verify the robot is in damping mode (limp) before sending commands
3. **Start in damping mode** - Always begin with `kp=0, kd=5` before sending position commands
4. **Joint limits** (from Unitree SDK):
   - Hip: ±1.047 rad (±60°)
   - Thigh: -0.663 to 2.966 rad (-38° to 170°)
   - Calf: -2.721 to -0.837 rad (-156° to -48°)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Control Stack                        │
│  ┌─────────────────┐    ┌──────────────────┐    ┌────────────┐  │
│  │ Joint Trajectory│───▶│  Hardware        │───▶│  Unitree   │  │
│  │   Controller    │    │  Interface       │    │  LowLevel  │  │
│  └─────────────────┘    └──────────────────┘    │  Driver    │  │
│         ▲                       ▲               └──────┬─────┘  │
│         │                       │                      │        │
│         │              ┌────────┴────────┐             │        │
│         │              │  Controller     │             ▼        │
│         └──────────────│    Manager      │      UDP (500Hz)     │
│                        └─────────────────┘         │            │
└────────────────────────────────────────────────────┼────────────┘
                                                     ▼
                                            ┌─────────────────┐
                                            │  Unitree Go1    │
                                            │  (LOWLEVEL UDP) │
                                            └─────────────────┘
```

---

## Installation

```bash
# Build the workspace
cd ~/ros2_ws
colcon build --packages-select unitree_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Docker builds (Jazzy & Kilted)

Both distros are built in Docker and verified end-to-end:

```bash
# Jazzy
docker build --network=host -f docker/jazzy/Dockerfile -t unitree_ros:jazzy .

# Kilted
docker build --network=host -f docker/kilted/Dockerfile -t unitree_ros:kilted .
```

Notes:
- The Dockerfiles use `rosdep install --from-paths src`; the rosdep step runs its own `apt-get update` so that newly published packages are always picked up (avoids stale cached apt indexes).
- `unitree_hardware_interface.xml` is copied into the image — it is required at configure time by `pluginlib_export_plugin_description_file` and referenced from `package.xml` via `${prefix}/unitree_hardware_interface.xml`.
- The hardware plugin is exported under the category `hardware_interface`, not `unitree_ros`:
  `pluginlib_export_plugin_description_file(hardware_interface unitree_hardware_interface.xml)`.
  This is required so the `hardware_interface::SystemInterface` ClassLoader discovers it.

---

## Control Methods

### Method 1: Direct Topic Control (Simple Testing)

For quick testing without `ros2_control`:

```bash
# Terminal 1: Start low-level driver
ros2 launch unitree_ros lowlevel.launch.py robot_ip:=192.168.123.161

# Terminal 2: Send commands via custom topics (debugging only)
ros2 topic pub /low_cmd unitree_ros/msg/LowCmd "motor_cmd: [...]"
```

**Topics:**
- `/joint_states` - `sensor_msgs/JointState` (standard ROS joint states)
- `/low_state` - `unitree_ros/msg/LowState` (full low-level state for debugging)

---

### Method 2: ros2_control + Standard Controllers (Recommended)

This uses the standard ROS2 Control pipeline with `joint_trajectory_controller`.

#### Launch the full stack:

```bash
# Terminal 1: Start hardware interface + controller manager + controllers
ros2 launch unitree_ros lowlevel_ros2_control.launch.py robot_ip:=192.168.123.161
```

This starts:
1. **Robot State Publisher** - Publishes the generated URDF to `/robot_description`
2. **Hardware Interface** (`unitree_hardware_interface`) - Talks to robot via UDP
3. **Controller Manager** (`ros2_control_node`) - Manages controllers
4. **Joint State Broadcaster** - Publishes `/joint_states`
5. **Joint Trajectory Controller** - Accepts trajectory commands

The launch generates a minimal URDF inline with real `<joint>` elements (with Go1 joint limits), `<link>` elements, and the `<ros2_control>` hardware block. The controller manager on Jazzy/Kilted reads `robot_description` from the `/robot_description` **topic** (published by `robot_state_publisher`), not from a node parameter. The URDF `<joint>` elements are also required so `enforce_command_limits` can import the joint limits.

#### Send joint trajectory commands:

```bash
# Send a trajectory to move joints
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
              'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
              'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
              'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']
points:
  - positions: [0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2, nanosec: 0}
"
```

#### Python example for sending trajectories:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_sender')
        self.pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.joint_names = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
        ]

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        # Stand pose (approximate)
        point.positions = [0.0, 0.9, -1.8] * 4
        point.velocities = [0.0] * 12
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info('Sent trajectory')

def main():
    rclpy.init()
    rclpy.spin(TrajectorySender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Method 3: Custom Controller (Advanced)

For custom control algorithms, implement a `ros2_control` controller:

```cpp
// MyCustomController.hpp
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

class MyCustomController : public controller_interface::ControllerInterface
{
  // Implement update() to read state interfaces and write command interfaces
};
```

Register in `plugin_description.xml` and load via controller manager.

---

## Joint Mapping

| ROS Joint Name | SDK Index | Description |
|----------------|-----------|-------------|
| `FR_hip_joint` | 0 | Front Right Hip |
| `FR_thigh_joint` | 1 | Front Right Thigh |
| `FR_calf_joint` | 2 | Front Right Calf |
| `FL_hip_joint` | 3 | Front Left Hip |
| `FL_thigh_joint` | 4 | Front Left Thigh |
| `FL_calf_joint` | 5 | Front Left Calf |
| `RR_hip_joint` | 6 | Rear Right Hip |
| `RR_thigh_joint` | 7 | Rear Right Thigh |
| `RR_calf_joint` | 8 | Rear Right Calf |
| `RL_hip_joint` | 9 | Rear Left Hip |
| `RL_thigh_joint` | 10 | Rear Left Thigh |
| `RL_calf_joint` | 11 | Rear Left Calf |

---

## Command Interfaces (per joint)

The hardware interface exposes these standard command interfaces:

| Interface | Description | Unit |
|-----------|-------------|------|
| `position` | Target position | rad |
| `velocity` | Target velocity | rad/s |
| `effort` | Feedforward torque | N⋅m |

**Control Law:** `τ = kp × (q_target - q_current) + kd × (dq_target - dq_current) + τ_ff`

The PD gains `kp` and `kd` are set globally via the `default_kp` / `default_kd` launch parameters (defaults: `60.0` and `3.0`).

---

## State Interfaces (per joint)

| Interface | Description | Unit |
|-----------|-------------|------|
| `position` | Current position | rad |
| `velocity` | Current velocity | rad/s |
| `effort` | Estimated torque | N⋅m |

---

## Configuration

### Launch Parameters (`lowlevel_ros2_control.launch.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `192.168.123.161` | Robot IP address |
| `robot_port` | `8082` | UDP port |
| `default_kp` | `60.0` | Default position gain |
| `default_kd` | `3.0` | Default velocity gain |

The controller configuration is loaded from `config/unitree_ros_control.yaml` inside the installed package.

### Controller Config (`config/unitree_ros_control.yaml`)

Key settings:
- `update_rate: 500` - Control loop frequency (must be 500Hz for Unitree)
- `command_interfaces: [position, velocity]` - Active command types
- `state_interfaces: [position, velocity, effort]` - Available state feedback

---

## Debugging Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Standard joint states |
| `/low_state` | `unitree_ros/msg/LowState` | Full low-level state (20 motors + IMU + BMS) |
| `/controller_manager/controller_state` | `controller_manager_msgs/msg/ControllerState` | Controller status |

---

## Troubleshooting

### "Cannot connect to robot"
- Verify IP: `ping 192.168.123.161`
- Check network interface: `ifconfig` (should have static IP on same subnet)
- Ensure robot is powered on and in low-level mode (L1+L2+Start)

The hardware interface refuses to initialize (`on_init` returns `ERROR`) when the robot IP is unreachable — this is intentional fail-fast behavior. The driver constructor pings the robot IP and throws if it is not reachable.

### "Controller manager waits for 'robot_description' topic"
- Since Jazzy/Kilted, `ros2_control_node` subscribes to the `/robot_description` **topic** instead of reading a `robot_description` parameter.
- The launch file includes `robot_state_publisher`, which publishes the generated URDF with `transient_local` QoS. Do not remove it.

### "Joint 'X' not found in URDF"
- `enforce_command_limits` requires every joint in the `<ros2_control>` block to exist as a real `<joint>` element in the URDF (with a `<limit>` child).
- The generated URDF in `lowlevel_ros2_control.launch.py` includes these joints and the Go1 limits (hip ±1.047 rad, thigh -0.663/2.966 rad, calf -2.721/-0.837 rad). If you write your own URDF, keep the `<joint>` elements.

### "Joint trajectory controller not active"
```bash
# Check controller status
ros2 control list_controllers

# Manually activate if needed
ros2 control set_controller_state joint_trajectory_controller active
```

### "Robot flails / moves unexpectedly"
- **STOP IMMEDIATELY** (Ctrl+C)
- Verify robot is suspended
- Start in damping mode: `driver_->set_damping_mode()`
- Check joint limits in your commands

### "CRC errors / communication failures"
- Ensure 500Hz control loop timing
- Check UDP buffer sizes
- Verify no other process using port 8082/8090

### Jazzy only: `ros2_control_node` dies with a Fast CDR symbol error
```
ros2_control_node: symbol lookup error: libpal_statistics_msgs__rosidl_typesupport_fastrtps_cpp.so:
  undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj
```
- This is an **upstream packaging issue** in the Jazzy apt repo (as of 2026-06): `pal_statistics_msgs` 2.7.0 was built against a Fast CDR ABI that provides `Cdr::serialize(unsigned int)`, but the installed `ros-jazzy-fastcdr` is 2.2.5, which only provides `Cdr::serialize(int)`.
- It occurs even for a stock `ros2_control_node` with no Unitree code involved, and does **not** affect Kilted.
- Tracking: ros2/rosidl_typesupport_fastrtps#126, eProsima/Fast-CDR#266.
- Workarounds: run the stack on Kilted, or wait for the Jazzy packages to be rebuilt against a consistent Fast CDR.

---

## Performance Notes

- **Control Loop**: 500Hz (2ms period) - hard real-time requirement
- **UDP Latency**: <1ms typical on direct Ethernet
- **Joint State Publish**: 500Hz (configurable via `publish_rate`)
- **Recommended PC**: Ubuntu 22.04/24.04 with RT kernel for best performance

---

## Migration from High-Level Control

| High-Level (Old) | Low-Level (New) |
|------------------|-----------------|
| `/cmd_vel` (Twist) | `/joint_trajectory_controller/joint_trajectory` |
| `stand_up` / `stand_down` services | Trajectory to stand/sit poses |
| `/joint_states` (from high_state) | `/joint_states` (from low_state - higher fidelity) |
| Gait modes (trot, walk) | Custom trajectories or gait controllers |

---

## References

- [Unitree Legged SDK v3.5.1](https://github.com/unitreerobotics/unitree_legged_sdk)
- [ROS2 Control Documentation](https://control.ros.org/)
- [Joint Trajectory Controller](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [unitree_ros2_to_real](https://github.com/unitreerobotics/unitree_ros2_to_real) - Official low-level examples