#ifndef KEYBOARD_COMMAND_INTERFACE_H
#define KEYBOARD_COMMAND_INTERFACE_H

#include <iostream>
#include <ncurses.h>
#include <thread>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class KeyboardListener : public rclcpp::Node
{
public:
    KeyboardListener();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    void listen_to_keyboard();
    std::string key_to_string(int ch);
    void publish_key_event(const std::string &key);
    bool enable;
};

#endif
