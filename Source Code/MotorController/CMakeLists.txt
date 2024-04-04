cmake_minimum_required(VERSION 3.5)
project(motor_controller)

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
add_executable(motor_controller_node src/motor_controller_node.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(motor_controller_node
  rclcpp
  std_msgs
)

# Link against wiringPi library
target_link_libraries(motor_controller_node wiringPi)

# Install the executable
install(TARGETS
  motor_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()
