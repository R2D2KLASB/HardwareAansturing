cmake_minimum_required(VERSION 3.22)
project(HardwareAansturing)

set(CMAKE_CXX_STANDARD 14)

include_directories(Infrastructuur-feature-working-nodes/src/battleship/include)
include_directories(Infrastructuur-feature-working-nodes/src/receive_node/include)
include_directories(Infrastructuur-feature-working-nodes/src/receive_node/include/receive_node)
include_directories(Infrastructuur-feature-working-nodes/src/send_node/include)
include_directories(Infrastructuur-feature-working-nodes/src/send_node/include/send_node)
include_directories(Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src)

add_executable(HardwareAansturing
        Infrastructuur-feature-working-nodes/src/battleship/include/display_node.hpp
        Infrastructuur-feature-working-nodes/src/battleship/include/ros_and_serial.hpp
        Infrastructuur-feature-working-nodes/src/battleship/include/SerialPort.h
        Infrastructuur-feature-working-nodes/src/battleship/include/SerialPortConstants.h
        Infrastructuur-feature-working-nodes/src/battleship/include/SerialStream.h
        Infrastructuur-feature-working-nodes/src/battleship/include/SerialStreamBuf.h
        Infrastructuur-feature-working-nodes/src/battleship/include/sound_player_node.hpp
        Infrastructuur-feature-working-nodes/src/battleship/include/xy_plotter_node.hpp
        Infrastructuur-feature-working-nodes/src/battleship/src/display_node.cpp
        Infrastructuur-feature-working-nodes/src/battleship/src/ros_and_serial.cpp
        Infrastructuur-feature-working-nodes/src/battleship/src/SerialPort.cpp
        Infrastructuur-feature-working-nodes/src/battleship/src/SerialStream.cpp
        Infrastructuur-feature-working-nodes/src/battleship/src/SerialStreamBuf.cpp
        Infrastructuur-feature-working-nodes/src/battleship/src/sound_player_node.cpp
        Infrastructuur-feature-working-nodes/src/battleship/src/xy_plotter_node.cpp
        Infrastructuur-feature-working-nodes/src/receive_node/include/receive_node/receive_node.hpp
        Infrastructuur-feature-working-nodes/src/receive_node/src/main.cpp
        Infrastructuur-feature-working-nodes/src/receive_node/src/receive_node.cpp
        Infrastructuur-feature-working-nodes/src/send_node/include/send_node/send_node.hpp
        Infrastructuur-feature-working-nodes/src/send_node/src/main.cpp
        Infrastructuur-feature-working-nodes/src/send_node/src/send_node.cpp
        Infrastructuur-feature-working-nodes/src/Speech/gcode_tools/string_to_gcode.cpp
        Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src/json.hpp
        Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src/pa_recorder.hpp
        Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src/publisher_member_function.cpp
        Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src/string_to_gcode.hpp
        Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src/tiny_gpio.h
        Infrastructuur-feature-working-nodes/src/Speech/ros2/speech_ros2/src/transcription_api.hpp)
