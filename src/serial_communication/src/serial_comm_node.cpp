#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <iostream>
#include <chrono>
#include <string>

using namespace std::chrono_literals;
using std::string;

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        // Adjust the serial port name and baud rate accordingly
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);

        try {
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            rclcpp::shutdown();
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        } else {
            return;
        }

        timer_ = this->create_wall_timer(1000ms, std::bind(&MotorController::communicate, this));
    }

    int readData()
    {
        string read_data = serial_.readline(1024, "\n");
        int received_data = std::stoi(read_data);
        RCLCPP_INFO(this->get_logger(), "Received: '%d'", received_data);
        return received_data;
    }

    void sendData(int data)
    {
        serial_.write(std::to_string(data) + "\n");
        RCLCPP_INFO(this->get_logger(), "Sent: '%d'", data);
    }

private:
    void communicate() {
        if (serial_.available()) {
            int received_data = this->readData();
            // Process the received data and send a response
            int send_data = received_data + 2; // Example processing
            this->sendData(send_data);
        } else {
            // Send some initial data or keep-alive message to Arduino
            sendData(10); // Sending an example value
        }
    }

    serial::Serial serial_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}
