#include "motor_controller.h"

MotorController::MotorController() : Node("motor_controller") 
{
    serial.setPort("/dev/ttyUSB0");
    serial.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial.setTimeout(timeout);

    try {
        serial.open();
    }
    catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port");
        rclcpp::shutdown();
    }

    if (serial.isOpen()) RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
    else return;

    subscription = this->create_subscription<motor_controller::msg::MotorCommand>(
        "/motor_commands", 10, std::bind(&MotorController::commandCallback, this, std::placeholders::_1)
    );
}

void MotorController::commandCallback(const motor_controller::msg::MotorCommand::SharedPtr msg)
{
    std::string command = msg->motor;  // use entire command string for convenience (to be rewritten properly)
    RCLCPP_INFO(this->get_logger(), "Received command: '%s'", command.c_str());

    // Send command to Arduino
    sendData(command);
}

int MotorController::readData() 
{
    std::string read_data = serial.readline(1024, "\n");
    int received_data = std::stoi(read_data);
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", received_data);
    return received_data;
}

void MotorController::sendData(std::string data) 
{
    serial.write(data + "\n");
    RCLCPP_INFO(this->get_logger(), "Sent: '%s'", data.c_str());
}

void MotorController::communicate() 
{
    if (serial.available()) {
        int received_data = this->readData();
        std::string send_data = std::to_string(received_data + 2); // example processing
        this->sendData(send_data);
    } 
    else 
    {
        // some some initial data or keep-alive message to arduino
        sendData(std::to_string(10)); // example value
    }
}

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}
