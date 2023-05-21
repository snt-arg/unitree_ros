# ROS 2 package for Controlling the Unitree Go1 Robot
This ROS2 package provides a driver for controlling the Unitree Go1 robot. The Unitree Go1 is a quadrupedal robot that can walk, run, jump, and perform other dynamic movements. With this driver, you can send commands to the robot via ROS topics such as `cmd_vel` and or receive robot sensors states such as `odometry` and `IMU` information.

This package provides a set of ROS2 topics which are defined in the [ROS Topics](#ros-topics) section, as well as a set of custom messages which were retrieved from  [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real) repository.


# unitree_ros
This is a ros2 package with the purpose of controlling an Unitree Go1 legged robot.
This package makes use of the [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) repositories.

# Dependencies
This package depends on ROS2, the `foxy` distribution, which in turn need Ubuntu `20.04`.

# Comming in the future

In the near future, plans to publish the raw data from the robot cameras will take place.

# ROS Topics

## Subscribers

- `/cmd_vel` [[geometry_mgs/msg/Twist.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)]: This is used by the driver to send the velocity commands to the robot.

- `/stand_up` [[std_msgs/msg/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]: Triggers the robot to stand up (not yet functional)

- `/stand_down` [[std_msgs/msg/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)]: Triggers the robot to stand down (not yet functional)

## Publishers

- `/odom` [[nav_msgs/msg/Odometry.msg](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)]: The odometry received from the robot is being published to this topic.

- `/imu` [[sensor_msgs/msg/IMU.msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)]: The IMU state received from the robot is being published to this topic.

- `/bms` [[unitree_ros/msg/bms.msg](https://github.com/snt-arg/unitree_ros/blob/main/msg/BmsState.msg)]: The battery state received from the robot is being published to this topic.
