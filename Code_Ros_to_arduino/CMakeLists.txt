cmake_minimum_required(VERSION 3.16)

project(Code_Ros_to_arduino)
set(CMAKE_CXX_STANDARD 14)
link_libraries(/home/deltarobot/clion/serialib-master/lib)

add_executable(Code_Ros_to_arduino
        RPI_send_string.cpp
        ros_node.cpp
        serialib-master/lib/serialib.cpp)

