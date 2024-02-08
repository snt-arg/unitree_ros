# Unitree Go1 ROS2 Driver

<p align="center">
    <img src="./docs/unitree_go1.png" width="30%"/>

## Description

A ROS2 package which can be used to control the legged robot **Unitree Go1**
using ROS topics.

This package acts has middleware between ROS2 and `unitree_legged_sdk`, which enables
anyone to control the robot with velocity commands as well as receive back the robot state. Furthermore, additional features are also available such as standing up/down the robot and use head LEDs for some status information.

More of the features can be found [here](#features).

> [!INFO]
> Only supports `unitree_legged_sdk` High level commands

</p>

For more information about the different topics this ROS package subscribes to or publishes, please refer to the [ROS Topics](#ros-topics) section.

## Table of Contents

<!--toc:start-->

- [ROS Distro Build Status](#ros-distro-build-status)
- [Description](#description)
- [Installation](#installation)
  - [Installation From Source](#installation-from-source)
- [Usage](#usage)
- [ROS Topics](#ros-topics)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
- [ROS Parameters](#ros-parameters)
- [Features](#features)
  - [Robot LED statuses](#robot-led-statuses)
  - [Low Battery Protection](#low-battery-protection)
  - [Obstacle Avoidance](#obstacle-avoidance)
- [Related Packages](#related-packages)
- [License](#license)
- [Contributions](#contributions)
- [Credits](#credits) - [Maintainers](#maintainers) - [Third-party Assets](#third-party-assets)
  <!--toc:end-->

## ROS Distro Build Status

| ROS Distro | Status                                                                                        |
| ---------- | --------------------------------------------------------------------------------------------- |
| **Iron**   | ![iron](https://github.com/snt-arg/unitree_ros/actions/workflows/iron_build.yaml/badge.svg)   |
| **Humble** | ![iron](https://github.com/snt-arg/unitree_ros/actions/workflows/humble_build.yaml/badge.svg) |
| **Foxy**   | ![iron](https://github.com/snt-arg/unitree_ros/actions/workflows/foxy_build.yaml/badge.svg)   |

## Installation

This package will be available soon in the ROS index. Thus, you can install it using `apt` (For now you can install it from source):

```bash
sudo apt install ros-[distro]-unitree-ros
```

> [!WARNING]
> If you are using foxy as your ROS distribution, you need to build from source

### Installation From Source

> [!NOTE]
> Make sure to clone it recursively, as depends on the `unitree_legged_sdk`.

```sh
mkdir -p ~/unitree_ws/src
cd ~/unitree_ws/src
git clone --recurse-submodules https://github.com/snt-arg/unitree_ros.git
```

Once you have cloned this repository, you will need to build it using Colcon.

```sh
cd ~/unitree_ws
source /opt/ros/[ros-distro]/setup.bash # or zsh if using the zsh shell!
colcon build --symlink-install
source install/setup.bash # or zsh if using the zsh shell!
```

After having build the workspace, you should now be able to use the driver to control your robot.

## Usage

Before using the driver, you will need to make a decision whether you want to control the robot
using a Wi-Fi connection or a wired connection. In case you go for a wired connection, you won't need
to do anything. In case you want to use the Wi-Fi connection, there are 2 options:

1. You can simply launch the driver and passing the `robot_ip` argument:

```bash
source ~/unitree_ws/install/setup.bash # or zsh if using the zsh shell!
ros2 launch unitree_ros unitree_driver_launch.py robot_ip:="192.168.12.1"
```

2. A configuration file is also available under `config/params.yaml`. You can change
   the `robot_ip` there too:

```bash
source ~/unitree_ws/install/setup.bash # or zsh if using the zsh shell!
ros2 launch unitree_ros unitree_driver_launch.py params_file:="path_to_your_params_file"
```

> [!INFO]
> In case, you want to change some of the other parameters available, such as the topic names, then you need to use the `config/params.yaml` file for that.

### ROS Topics

#### Subscribed Topics

| Topic name    | Message Type                                                                                      | Description                                                                                             |
| ------------- | ------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| `cmd_vel`     | [geometry_mgs/msg/Twist.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) | This is used by the driver to receive velocity commands and send the appropriate commands to the robot. |
| `/stand_up`   | [std_msgs/msg/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)         | Triggers the robot to stand up                                                                          |
| `/stand_down` | [std_msgs/msg/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)         | Triggers the robot to stand down                                                                        |

#### Published Topics

| Topic name       | Message Type                                                                                              | Description                                                                  |
| ---------------- | --------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| `/odom`          | [nav_msgs/msg/Odometry.msg](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)            | The odometry state received from the robot is being published to this topic. |
| `/imu`           | [sensor_msgs/msg/IMU.msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)                | The IMU state received from the robot is being published to this topic.      |
| `/bms_state`     | [unitree_ros/msg/bms.msg](https://github.com/snt-arg/unitree_ros/blob/main/msg/BmsState.msg)              | The battery state received from the robot is being published to this topic.  |
| `/sensor_ranges` | [unitree_ros/msg/SensorRanges.msg](https://github.com/snt-arg/unitree_ros/blob/main/msg/SensorRanges.msg) | The battery state received from the robot is being published to this topic.  |

## ROS Parameters

| Parameter Name                | Default value   | Description                                                                                              |
| ----------------------------- | --------------- | -------------------------------------------------------------------------------------------------------- |
| `ns`                          | -               | Name space that should be given to robot driver                                                          |
| `robot_ip`                    | 192.168.123.161 | Robot IP that should be used to establish the UDP connection. For a Wi-Fi conntection use `192.168.12.1` |
| `robot_target_port`           | 8082            | The port that should be used to communicate with the robot.                                              |
| `cmd_vel_topic_name`          | /cmd_vel        | Topic name that should be used for subscribing to velocity commands                                      |
| `odom_topic_name`             | /odom           | Topic name that should be used for publishing the odometry state                                         |
| `imu_topic_name`              | /imu            | Topic name that should be used for publishing the IMU state                                              |
| `bms_state_topic_name`        | /bms_state      | Topic name that should be used for publishing the battery management state, such as battery level.       |
| `imu_frame_id`                | imu             | Frame id that should be used for the IMU frame                                                           |
| `odom_frame_id`               | odom            | Frame id that should be used for the odometry frame                                                      |
| `odom_child_frame_id`         | base_link       | Frame id of the body of the robot                                                                        |
| `use_obstacle_avoidance`      | false           | Enables (true) or disables (false) the robot obstacle avoidance.                                         |
| `low_batt_shutdown_threshold` | 20              | Battery threshold for when to stop the robot from moving, in case the battery is below                   |

## Features

> [!INFO]
> In the next version, joint states will also be available.

### Robot LED statuses

The robot has a few predetermined LED statuses, which are useful to give some information to
anyone using the robot.
The following statuses are available:

- **Green Light**: Ready status

    <img src="./docs/ready_status.gif" width="150" height="150"/>

- **White Light**: Idle status

    <img src="./docs/idle_status.gif" width="150" height="150"/>

- **Blue Light**: Moving status

    <img src="./docs/moving_status.gif" width="150" height="150"/>

- **Yellow Light**: Low battery _(< 30 %)_

    <img src="./docs/low_battery_status.gif" width="150" height="150"/>

### Low Battery Protection

By specifying a low battery threshold using the parameters file (`low_batt_shutdown_threshold`), the driver will stop the robot
from moving and will stand it down. _By default, the low battery threshold value is set to 20%._

### Obstacle Avoidance

The robot has an obstacle avoidance mode. However, this mode is not enabled by default. Therefore,
this driver allows you to enable it using the parameters file (`use_obstacle_avoidance`). _By default, this is
set to false_

## Related Packages

- [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)

## License

This project is licensed under the SnT academic license - see the [LICENSE](https://github.com/snt-arg/unitree_ros/blob/main/LICENSE) for more details.

## Contributions

Contributions are welcome! If you have any suggestions, bug reports, or feature requests,
please create a new issue or pull request.

## Credits

This package was developed for the Autonomous Robotics Group (ARG) from the University of Luxembourg.

#### Maintainers

- [Pedro Soares](https://www.github.com/PedroS235)

#### Third-party Assets

- [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk): This is the SDK provided by the Unitree robotics. It is being used to send/receive all the High level commands to/from the robot.

- **`faceLightSDK_nano`**: This is the SDK that can be found on the internal computers of the Unitree Go1. It has been ported to this ROS package with the goal of being able to control the face LEDs, which are used to
  give some robot statuses.
