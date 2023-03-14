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


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include unitree_legged_sdk
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(include 
                   ${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/include)

add_executable(unitree_ros src/unitree_ros_node.cpp)
target_link_libraries(unitree_ros ${EXTRA_LIBS})
ament_target_dependencies(unitree_ros rclcpp std_msgs)

install(
  DIRECTORY include
  DIRECTORY  ${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/include
  DESTINATION include
)


install(TARGETS 
  unitree_ros 
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()
