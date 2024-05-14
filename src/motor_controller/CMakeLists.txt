cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_library(SERIAL_LIB serial)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/MotorCommand.msg"
)

if(NOT SERIAL_LIB)
  message(FATAL_ERROR "Serial library not found")
endif()

include_directories(include)

add_executable(motor_controller_node src/motor_controller.cpp)
ament_target_dependencies(motor_controller_node rclcpp)
rosidl_target_interfaces(motor_controller_node ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(motor_controller_node ${SERIAL_LIB})

install(TARGETS
  motor_controller_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()