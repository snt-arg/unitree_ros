#include <rclcpp/rclcpp.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_ros/unitree_driver_ros.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeDriverRos>());
    rclcpp::shutdown();
    return 0;
}
