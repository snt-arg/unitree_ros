cmake_minimum_required(VERSION 3.5)
project(unitree_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# add_library(unitree_ros src/unitree_ros.cpp)

add_executable(unitree_ros src/unitree_ros_node.cpp)
ament_target_dependencies(unitree_ros rclcpp std_msgs)

install(
  DIRECTORY include/unitree_ros
  DESTINATION include
)


install(TARGETS 
  unitree_ros 
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()
