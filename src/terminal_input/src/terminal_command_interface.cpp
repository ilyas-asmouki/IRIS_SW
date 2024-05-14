#include <iostream>
#include <vector>
#include "terminal_command_interface.h"

TerminalCommandInterface::TerminalCommandInterface() : Node("terminal_command_interface")
{
    command_publisher = this->create_publisher<motor_controller::msg::MotorCommand>("/motor_commands", 10);
    keyhold_publisher = this->create_publisher<motor_controller::msg::MotorCommand>("/key_commands", 10);
    std::thread([this]() { this->listen(); }).detach();
}

void TerminalCommandInterface::listen()
{
    std::string line;
    std::cout << "Connection to node successful. Enter commands (e.g., 'fl 149 rr 19 fr 218'): " << std::endl;
    while (std::getline(std::cin, line))
    {
        std::istringstream iss(line);
        std::string motor;
        int velocity;
        std::vector<std::string> used_directions = {};
        bool invalid = false;
        while (iss >> motor) {
            if (motor != "fr" and motor != "fl" and motor != "rr" and motor != "rl") {
                std::cout << "Invalid direction" << std::endl;
                invalid = true;
                break;
            } 
            for (const auto& direction : used_directions) {
                if (direction == motor) {
                    invalid = true;
                    break;
                }
            }
            if (repeated) {
                std::cout << "Direction " << motor << " repeated" << std::endl;
                invalid = true;
                break;
            }
            used_directions.push_back(motor);
            if (not (iss >> velocity)) {
                std::cout << "Missing velocity for " << motor << std::endl;
                invalid = true;
                break;
            }
            if (not invalid) {
                auto message = motor_controller::msg::MotorCommand();
                message.motor = motor;
                message.velocity = velocity;
                command_publisher->publish(message);
            }
        }
        std::cout << "Enter commands: " << std::endl;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerminalCommandInterface>());
    rclcpp::shutdown();
    return 0;
}
