cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# used for some quaternion-angle conversions
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(controller src/controller.cpp src/util.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

target_include_directories(controller PRIVATE inc)

install(TARGETS
    controller
    DESTINATION lib/${PROJECT_NAME})

ament_package()