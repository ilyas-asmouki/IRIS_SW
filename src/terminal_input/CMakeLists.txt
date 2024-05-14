cmake_minimum_required(VERSION 3.8)
project(terminal_input)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_controller REQUIRED)

include_directories(include ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(terminal_command_interface src/terminal_command_interface.cpp)
ament_target_dependencies(terminal_command_interface
  rclcpp
  motor_controller
)

install(TARGETS
  terminal_command_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
