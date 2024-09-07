#ifndef TERMINAL_COMMAND_INTERFACE_H
#define TERMINAL_COMMAND_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sstream>
#include <thread>
#include "motor_controller/msg/motor_command.hpp"

class TerminalCommandInterface : public rclcpp::Node
{
public:
    TerminalCommandInterface();
private:
    rclcpp::Publisher<motor_controller::msg::MotorCommand>::SharedPtr command_publisher;
    rclcpp::Publisher<motor_controller::msg::MotorCommand>::SharedPtr keyhold_publisher;
    void listen();
    void sendMotorCommand(const std::string &motor, int velocity);
};

#endif