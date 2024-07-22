/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#include <iostream>
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
    msg.orientation.x = imu.quaternion[1];
    msg.orientation.y = imu.quaternion[2];
    msg.orientation.z = imu.quaternion[3];
    msg.orientation.w = imu.quaternion[0];

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

void serialize(unitree_ros::msg::SensorRanges& msg, const sensor_ranges_t radar) {
    msg.front = radar.front;
    msg.left = radar.left;
    msg.right = radar.right;
}

void serialize(sensor_msgs::msg::JointState& msg,
               const std::array<UNITREE_LEGGED_SDK::MotorState, 12> motor_states) {
    msg.name = {"front_right_hip",
                "front_right_tight",
                "front_right_calf",
                "front_left_hip",
                "front_left_tight",
                "front_left_calf",
                "back_right_hip",
                "back_right_tight",
                "back_right_calf",
                "back_left_hip",
                "back_left_tight",
                "back_left_calf"};

    int num_joints = 12;
    msg.position.resize(num_joints);
    msg.velocity.resize(num_joints);
    msg.effort.resize(num_joints);

    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            int idx = leg + joint;
            msg.position[idx] = motor_states[idx].q;
            msg.velocity[idx] = motor_states[idx].dq;
            msg.effort[idx] = motor_states[idx].tauEst;
        }
    }
}
