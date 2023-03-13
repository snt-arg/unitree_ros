#include <rclcpp/rclcpp.hpp>

class UnitreeRos: public rclcpp::Node{
    public:
        UnitreeRos() : Node("unitree_ros_node"){
            RCLCPP_INFO(this->get_logger(), "Node is alive");
        }
};