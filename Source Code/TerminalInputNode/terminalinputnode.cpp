#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>

class TerminalInputNode : public rclcpp::Node {
public:
  TerminalInputNode() : Node("terminal_input_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("motor_commands", 10);
  }

  void run() {
    std::string input;
    std_msgs::msg::String msg;
    while (rclcpp::ok()) {
      std::getline(std::cin, input);
      if (!input.empty()) {
        msg.data = input;
        publisher_->publish(msg);
      }
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerminalInputNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
