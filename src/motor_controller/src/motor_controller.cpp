#include "motor_controller.h"

MotorController::MotorController() : Node("motor_controller") 
{
    fr_velocity = 0;
    fl_velocity = 0;
    rr_velocity = 0;
    rl_velocity = 0;
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

    terminal_subscription = this->create_subscription<motor_controller::msg::MotorCommand>(
        "/motor_commands", 10, std::bind(&MotorController::terminal_callback, this, std::placeholders::_1)
    );

    keyboard_subscription = this->create_subscription<std_msgs::msg::String>(
        "/keyboard_events", 10, std::bind(&MotorController::keyboard_callback, this, std::placeholders::_1)
    );
}

void MotorController::terminal_callback(const motor_controller::msg::MotorCommand::SharedPtr msg)
{
    std::string command = msg->motor;  // use entire command string for convenience (to be rewritten properly)
    RCLCPP_INFO(this->get_logger(), "Received command: '%s'", command.c_str());

    // Send command to Arduino
    send_data(command);
}

void MotorController::keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "w") {
        fr_velocity = 255;
        fl_velocity = 255;
        rr_velocity = 255;
        rl_velocity = 255;
    } else if (msg->data == "s") {
        fr_velocity = -255;
        fl_velocity = -255;
        rr_velocity = -255;
        rl_velocity = -255;
    } else if (msg->data == "a") {
        fr_velocity = 255;
        fl_velocity = -255;
        rr_velocity = 255;
        rl_velocity = -255;
    } else if (msg->data == "d") {
        fr_velocity = -255;
        fl_velocity = 255;
        rr_velocity = -255;
        rl_velocity = 255;
    } else if (msg->data == "q") {
        fr_velocity = 255;
        fl_velocity = 128;
        rr_velocity = 255;
        rl_velocity = 128;
    } else if (msg->data == "e") {
        fr_velocity = 128;
        fl_velocity = 255;
        rr_velocity = 128;
        rl_velocity = 255;
    } else if (msg->data == "z") {
        fr_velocity = -255;
        fl_velocity = -128;
        rr_velocity = -255;
        rl_velocity = -128;
    } else if (msg->data == "c") {
        fr_velocity = -128;
        fl_velocity = -255;
        rr_velocity = -128;
        rl_velocity = -255;
    } else if (msg->data == "nothing") {
        fr_velocity = 0;
        fl_velocity = 0;
        rr_velocity = 0;
        rl_velocity = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Received keystroke: '%s'", msg->data.c_str());

    std::string command_to_send = std::string("fr ")  + std::to_string(fr_velocity) 
                                + std::string(" fl ") + std::to_string(fl_velocity)
                                + std::string(" rr ") + std::to_string(rr_velocity)
                                + std::string(" rl ") + std::to_string(rl_velocity);

    send_data(command_to_send);
}

int MotorController::read_data() 
{
    std::string read_data = serial.readline(1024, "\n");
    int received_data = std::stoi(read_data);
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", received_data);
    return received_data;
}

void MotorController::send_data(std::string data) 
{
    serial.write(data + "\n");
    RCLCPP_INFO(this->get_logger(), "Sent: '%s'", data.c_str());
}

// void MotorController::communicate() 
// {
//     if (serial.available()) {
//         int received_data = this->read_data();
//         // Process the received data and send a response
//         std::string send_data = std::to_string(received_data + 2); // Example processing
//         this->send_data(send_data);
//     } 
//     else 
//     {
//         // Send some initial data or keep-alive message to Arduino
//         send_data(std::to_string(10)); // Sending an example value
//     }
// }

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}
