cmake_minimum_required(VERSION 3.5)
project(terminal_input_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(terminal_input_node src/terminal_input_node.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(terminal_input_node
  rclcpp
  std_msgs
)

# Install the executable
install(TARGETS
  terminal_input_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()
