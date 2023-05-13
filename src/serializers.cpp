#include <unitree_ros/serializers.hpp>

void serialize(nav_msgs::msg::Odometry& msg, const odom_t odom) {
    msg.pose.pose.position.x = odom.pose.position.x;
    msg.pose.pose.position.y = odom.pose.position.y;
    msg.pose.pose.position.z = odom.pose.position.z;
    msg.pose.pose.orientation.x = odom.pose.orientation.x;
    msg.pose.pose.orientation.y = odom.pose.orientation.y;
    msg.pose.pose.orientation.z = odom.pose.orientation.z;
    msg.pose.pose.orientation.w = odom.pose.orientation.w;
    msg.twist.twist.linear.x = odom.velocity.x;
    msg.twist.twist.linear.y = odom.velocity.y;
    msg.twist.twist.angular.z = odom.velocity.yaw;
}

void serialize(sensor_msgs::msg::Imu& msg, const UNITREE_LEGGED_SDK::IMU imu) {
    msg.orientation.x = imu.quaternion[0];
    msg.orientation.y = imu.quaternion[1];
    msg.orientation.z = imu.quaternion[2];
    msg.orientation.w = imu.quaternion[3];

    msg.angular_velocity.x = imu.gyroscope[0];
    msg.angular_velocity.y = imu.gyroscope[1];
    msg.angular_velocity.z = imu.gyroscope[2];

    msg.linear_acceleration.x = imu.accelerometer[0];
    msg.linear_acceleration.y = imu.accelerometer[1];
    msg.linear_acceleration.z = imu.accelerometer[2];

    msg.orientation_covariance[0] = imu.rpy[0];
    msg.orientation_covariance[1] = imu.rpy[1];
    msg.orientation_covariance[2] = imu.rpy[2];
}

void serialize(unitree_ros::msg::BmsState& msg,
               const UNITREE_LEGGED_SDK::BmsState bms) {
    msg.soc = bms.SOC;
    msg.bms_status = bms.bms_status;
    msg.cell_vol = bms.cell_vol;
    msg.version_h = bms.version_h;
    msg.version_l = bms.version_l;
    msg.cycle = bms.cycle;
    msg.mcu_ntc = bms.MCU_NTC;
    msg.bq_ntc = bms.BQ_NTC;
    msg.current = bms.current;
}
