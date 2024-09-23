cmake_minimum_required(VERSION 3.8)
project(terminal_input)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Curses REQUIRED)
find_package(motor_controller REQUIRED)

include_directories(include ${CMAKE_CURRENT_SOURCE_DIR})

# Define the executables
add_executable(terminal_command_interface src/terminal_command_interface.cpp)
add_executable(keyboard_command_interface src/keyboard_command_interface.cpp)

# Link dependencies for each executable
ament_target_dependencies(terminal_command_interface
  rclcpp
  motor_controller
)
ament_target_dependencies(keyboard_command_interface rclcpp std_msgs)
target_link_libraries(keyboard_command_interface ${CURSES_LIBRARIES})

install(TARGETS
  terminal_command_interface
  keyboard_command_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
