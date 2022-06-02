cmake_minimum_required(VERSION 3.22)
project(VirtualPlotter)

set(CMAKE_CXX_STANDARD 14)
set(SFML_DIR "C:/SOFTWARE_TI/SFML-2.5.1-32/lib/cmake/SFML")

find_package(SFML REQUIRED COMPONENTS audio network graphics window system)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(.)
include_directories(Library/JSON/json)
include_directories(${SFML_INCLUDE_DIR})

add_executable(VirtualPlotter
        Library/JSON/json/json-forwards.h
        Library/JSON/json/json.h
        Library/JSON/jsoncpp.cpp
        ColorPicker.cpp
        ColorPicker.hpp
        ColorWheel.cpp
        ColorWheel.hpp
        ConfigState.cpp
        ConfigState.hpp
        DataTypes.hpp
        Definitions.hpp
        Exceptions.hpp
        Game.cpp
        Game.hpp
        GameState.hpp
        InputManager.cpp
        InputManager.hpp
        JsonManager.cpp
        JsonManager.hpp
        main.cpp
        MainState.cpp
        MainState.hpp
        plotter.cpp
        plotter.hpp
        StateMachine.cpp
        StateMachine.hpp
        StringEditors.hpp
        VirtualPlotter.cpp
        VirtualPlotter.hpp)

target_link_libraries(VirtualPlotter sfml-audio sfml-network sfml-graphics sfml-window sfml-system)