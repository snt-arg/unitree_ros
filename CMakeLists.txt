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


find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------------
# - External Libraries Section -
# ------------------------------

# unitree_legged_sdk Library
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/lib/cpp/${ARCH})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils/faceLightSDK_Nano/lib)
set(EXTRA_LIBS -pthread unitree_legged_sdk faceLight_SDK_amd64)
set(CMAKE_CXX_FLAGS "-O3 -march=native -Wall -fPIC")

include_directories(include 
                   ${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/include)
include_directories(include 
                   ${CMAKE_SOURCE_DIR}/utils/faceLightSDK_Nano/include)


# --------------------------------
# - Message Generatation Section -
# --------------------------------
              
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsState.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs
)

# -----------------------
# - Build Section -
# -----------------------

set(DEPENDENCIES rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

add_executable(unitree_driver src/unitree_ros_node.cpp src/unitree_ros.cpp src/unitree_driver.cpp src/serializers.cpp)
target_link_libraries(unitree_driver ${EXTRA_LIBS})
ament_target_dependencies(unitree_driver ${DEPENDENCIES})
rosidl_target_interfaces(unitree_driver ${PROJECT_NAME} "rosidl_typesupport_cpp")



# -------------------
# - Install Section -
# -------------------

install(
  DIRECTORY include ${CMAKE_SOURCE_DIR}/utils/unitree_legged_sdk/include ${CMAKE_SOURCE_DIR}/utils/faceLightSDK_Nano/include
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  unitree_driver
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()
