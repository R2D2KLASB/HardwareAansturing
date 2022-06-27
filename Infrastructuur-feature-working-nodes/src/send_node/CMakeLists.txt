cmake_minimum_required(VERSION 3.5)
project(send_node)

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

# add sources
#file(GLOB_RECURSE SOURCES "src/*")

# add the executable
#add_executable(${PROJECT_NAME} ${SOURCES})
add_executable(send_node src/main.cpp)
ament_target_dependencies(send_node rclcpp std_msgs)

install(TARGETS
  send_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
