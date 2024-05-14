#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "motor_controller/msg/motor_command.hpp"

using namespace std::chrono_literals;

class MotorController : public rclcpp::Node 
{
    public:
        MotorController();
        int readData();
        void sendData(std::string data);

    private:
        void communicate();
        serial::Serial serial;
        rclcpp::Subscription<motor_controller::msg::MotorCommand>::SharedPtr subscription;
        void commandCallback(const motor_controller::msg::MotorCommand::SharedPtr msg);
};