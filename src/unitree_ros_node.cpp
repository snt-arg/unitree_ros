#include <rclcpp/rclcpp.hpp>
#include "unitree_ros/unitree_ros.hpp"
#include "unitree_legged_sdk.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeRos>());
    rclcpp::shutdown();
    return 0;
}
