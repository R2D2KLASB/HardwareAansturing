cmake_minimum_required(VERSION 3.5)
project(rpi_serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(listener src/subscriber_member_function.cpp src/rpi_serial.hpp src/serialib/serialib.cpp)
target_include_directories(listener PUBLIC include/serialib)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS listener DESTINATION lib/${PROJECT_NAME})

ament_package()
