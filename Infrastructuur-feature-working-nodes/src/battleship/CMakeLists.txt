cmake_minimum_required(VERSION 3.5)
project(battleship)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lserial -lpthread)
endif()

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# The path to our header files:
# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)
include_directories(include/)

# add sources
#file(GLOB_RECURSE SOURCES "src/*")

# add the executable
#add_executable(${PROJECT_NAME} ${SOURCES})

add_executable(ros_and_serial src/ros_and_serial.cpp src/SerialStream.cpp src/SerialStreamBuf.cpp src/SerialPort.cpp)
ament_target_dependencies(ros_and_serial rclcpp std_msgs)

add_executable(sound_player src/sound_player_node.cpp )
ament_target_dependencies(sound_player rclcpp std_msgs)

install(TARGETS
    ros_and_serial
    sound_player
    DESTINATION lib/${PROJECT_NAME})

ament_package()
