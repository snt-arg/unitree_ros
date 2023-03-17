#include "unitree_ros/unitree_driver_ros.hpp"

#include "unitree_ros/conversion.hpp"

using namespace std::chrono_literals;

// TODO:
//  [] publish the battery state

UnitreeDriverRos::UnitreeDriverRos()
    : Node("unitree_driver_node"),
      robotUDPConnection(robotLocalPort,
                         sizeof(robotHighCmd),
                         sizeof(robotHighState),
                         false,
                         UNITREE_LEGGED_SDK::RecvEnum::nonBlock,
                         true) {
    readParams();

    robotUDPConnection.SetIpPort(robotIP.c_str(), robotTargetPort);
    robotUDPConnection.InitCmdData(robotHighCmd);

    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopicName,
        10,
        std::bind(&UnitreeDriverRos::cmdVelCallback, this, std::placeholders::_1));

    imuPub = create_publisher<sensor_msgs::msg::Imu>(imuTopicName, 10);
    odomPub = create_publisher<nav_msgs::msg::Odometry>(odometryTopicName, 10);

    robotStateTimer = create_wall_timer(
        50ms, std::bind(&UnitreeDriverRos::robotStateTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Unitree Driver Node is ready!");
}

void UnitreeDriverRos::readParams() {
    RCLCPP_INFO(get_logger(), "Reading ROS parameters...");

    declare_parameter<std::string>("robot_ip", "192.168.12.1");
    declare_parameter<int>("robot_target_port", 8082);
    declare_parameter<std::string>("cmd_vel_topic_name", "cmd_vel");
    declare_parameter<std::string>("odometry_topic_name", "/odom");
    declare_parameter<std::string>("odometry_frame_id", "Odometry");
    declare_parameter<std::string>("imu_topic_name", "/imu");
    declare_parameter<std::string>("imu_frame_id", "IMU");

    get_parameter("robot_ip", robotIP);
    get_parameter("robot_target_port", robotTargetPort);
    get_parameter("cmd_vel_topic_name", cmdVelTopicName);
    get_parameter("odometry_topic_name", odometryTopicName);
    get_parameter("odometry_frame_id", odometryFrameId);
    get_parameter("imu_topic_name", imuTopicName);
    get_parameter("imu_frame_id", imuFrameId);

    RCLCPP_INFO(get_logger(), "Finished reading ROS parameters!");
}

void UnitreeDriverRos::robotStateTimerCallback() {
    // Receive state from robot
    robotUDPConnection.Recv();
    robotUDPConnection.GetRecv(robotHighState);

    rclcpp::Time now = this->get_clock()->now();

    sensor_msgs::msg::Imu imuStateMsg = generateImuMsg(robotHighState, now, imuFrameId);
    nav_msgs::msg::Odometry odometryStateMsg =
        generateOdometryMsg(robotHighState, now, odometryFrameId);

    odomPub->publish(odometryStateMsg);
    imuPub->publish(imuStateMsg);
}

void UnitreeDriverRos::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    robotHighCmd = rosMsg2Cmd(msg);
    robotUDPConnection.SetSend(robotHighCmd);
    robotUDPConnection.Send();
    RCLCPP_INFO(get_logger(), "Received cmd");
}
