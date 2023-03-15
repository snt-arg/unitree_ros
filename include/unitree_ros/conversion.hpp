#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <geometry_msgs/msg/twist.hpp>
#include <unitree_ros/msg/bms_cmd.hpp>
#include <unitree_ros/msg/high_cmd.hpp>

inline UNITREE_LEGGED_SDK::BmsCmd rosMsg2Cmd(const unitree_ros::msg::BmsCmd &msg) {
    UNITREE_LEGGED_SDK::BmsCmd cmd;

    cmd.off = msg.off;

    for (int i(0); i < 3; i++) {
        cmd.reserve[i] = msg.reserve[i];
    }

    return cmd;
}

inline UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(
    const unitree_ros::msg::HighCmd::SharedPtr &msg) {
    UNITREE_LEGGED_SDK::HighCmd cmd;

    for (int i(0); i < 2; i++) {
        cmd.head[i] = msg->head[i];
        cmd.SN[i] = msg->sn[i];
        cmd.version[i] = msg->version[i];
        cmd.position[i] = msg->position[i];
        cmd.velocity[i] = msg->velocity[i];
    }

    for (int i(0); i < 3; i++) {
        cmd.euler[i] = msg->euler[i];
    }

    for (int i(0); i < 4; i++) {
        cmd.led[i].r = msg->led[i].r;
        cmd.led[i].g = msg->led[i].g;
        cmd.led[i].b = msg->led[i].b;
    }

    for (int i(0); i < 40; i++) {
        cmd.wirelessRemote[i] = msg->wireless_remote[i];
    }

    cmd.levelFlag = msg->level_flag;
    cmd.frameReserve = msg->frame_reserve;
    cmd.bandWidth = msg->bandwidth;
    cmd.mode = msg->mode;
    cmd.gaitType = msg->gait_type;
    cmd.speedLevel = msg->speed_level;
    cmd.footRaiseHeight = msg->foot_raise_height;
    cmd.bodyHeight = msg->body_height;
    cmd.yawSpeed = msg->yaw_speed;
    cmd.reserve = msg->reserve;
    cmd.crc = msg->crc;

    cmd.bms = rosMsg2Cmd(msg->bms);

    return cmd;
}

inline UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    cmd.velocity[0] = msg->linear.x;
    cmd.velocity[1] = msg->linear.y;
    cmd.yawSpeed = msg->angular.z;

    cmd.mode = 2;
    cmd.gaitType = 1;

    return cmd;
}

#endif  // !CONVERSION_HPP
