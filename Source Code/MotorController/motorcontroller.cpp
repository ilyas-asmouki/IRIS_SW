#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <wiringSerial.h>
#include <sstream>

class MotorController : public rclcpp::Node {
public:
  MotorController() : Node("motor_controller") {
    serial_fd = serialOpen("/dev/ttyAMA0", 9600);
    if (serial_fd == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
    }

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "motor_commands", 10,
        std::bind(&MotorController::commandCallback, this, std::placeholders::_1));
  }

  ~MotorController() {
    if (serial_fd != -1) {
      serialClose(serial_fd);
    }
  }

private:
  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::istringstream iss(msg->data);
    std::string motor;
    int speed;
    if (!(iss >> motor >> speed)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid command format");
      return;
    }

    std::string serialCommand = motor + " " + std::to_string(speed) + "\n";
    serialPuts(serial_fd, serialCommand.c_str());
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", serialCommand.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int serial_fd;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
