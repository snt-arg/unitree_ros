# ROS 2 package for Controlling the Unitree Go1 Robot
This ROS2 package provides a driver for controlling the Unitree Go1 robot. The Unitree Go1 is a quadrupedal robot that can walk, run, jump, and perform other dynamic movements. With this driver, you can send commands to the robot via ROS topics such as `cmd_vel` and or receive robot sensors states such as `odometry` and `IMU` information.

For more informaion about the different topics this ROS package subscribes or publishes, please refer to [ROS Topics](#ros-topics) section.

## Dependencies
This package depends on ROS2, the `foxy` distribution, which in turn need Ubuntu `20.04`. (No testing has been made on ROS Humble, so no promises if this driver works)

- [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk): This is the SDK provided by the Unitree robotics. It is being used to send/receive all the High level commands to/from the robot.

- [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real): This is a ROS1 package provided by the Unitree robotics. The ROS messages have been adapted for ROS2 support.

- **faceLightSDK_nano**: This is the SDK that can be found on the internal computers of the Unitree Go1. It has been ported to this ROS package with the goal of begin able of controlling the face LEDs, which are used to 
give some robot status.

## Comming in the future

In the near future, plans to publish the raw data from the robot cameras will take place.

## Robot status

The robot has a few predetermined status, which are useful to give some information to anyone using the robot.
The following status are available:

- Green Light: Ready status
- White Light: Iddle status
- Blue Light: Moving status
- Yellow Light: Low battery  (< 30 %)
- Red Light: Any internal error (Not yet implemented)

## ROS Topics

### Subscribers

- `/cmd_vel` [[geometry_mgs/msg/Twist.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)]: This is used by the driver to send the velocity commands to the robot.

- `/stand_up` [[std_msgs/msg/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]: Triggers the robot to stand up (not yet functional)

- `/stand_down` [[std_msgs/msg/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]: Triggers the robot to stand down (not yet functional)

### Publishers

- `/odom` [[nav_msgs/msg/Odometry.msg](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)]: The odometry received from the robot is being published to this topic.

- `/imu` [[sensor_msgs/msg/IMU.msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)]: The IMU state received from the robot is being published to this topic.

- `/bms` [[unitree_ros/msg/bms.msg](https://github.com/snt-arg/unitree_ros/blob/main/msg/BmsState.msg)]: The battery state received from the robot is being published to this topic.
