#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "motor_controller/msg/motor_command.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MotorController : public rclcpp::Node 
{
public:
    MotorController();
    int read_data();
    void send_data(std::string data);

private:
    // void communicate();
    int fr_velocity;
    int fl_velocity;
    int rr_velocity;
    int rl_velocity;
    serial::Serial serial;
    rclcpp::Subscription<motor_controller::msg::MotorCommand>::SharedPtr terminal_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscription;
    void terminal_callback(const motor_controller::msg::MotorCommand::SharedPtr msg);
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg);
};
