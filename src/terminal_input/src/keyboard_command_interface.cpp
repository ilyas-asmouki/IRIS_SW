#include "keyboard_command_interface.h"

KeyboardListener::KeyboardListener() : Node("keyboard_listener") {
    enable = true;
    publisher = this->create_publisher<std_msgs::msg::String>("/keyboard_events", 10);
    std::thread([this]() { this->listen_to_keyboard(); }).detach();
}

void KeyboardListener::listen_to_keyboard() {
    const int timeout_duration_ms = 200;
    const int time_between_key_strokes_ms = 200;
    initscr();              // init ncurses
    cbreak();               // disable line buffering
    noecho();               // do not echo keystrokes
    timeout(timeout_duration_ms);
    keypad(stdscr, TRUE);   // enable special keys

    auto last_publish_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        int ch = getch();   // get pressed key

        if (ch != ERR) {
            auto now = std::chrono::steady_clock::now();
            auto duration_since_last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time);

            if (duration_since_last_publish.count() >= time_between_key_strokes_ms) {
                std::string key_str = key_to_string(ch);
                if (key_str == "p") {
                    enable = not enable;
                    RCLCPP_INFO(this->get_logger(), "Keyboard Command: %s\n", (enable ? "Enabled." : "Disabled."));
                }
                else if (enable and not key_str.empty()) {
                    publish_key_event(key_str);
                    last_publish_time = now;
                }
            }
        } else {
            if (enable and std::chrono::steady_clock::now() - last_publish_time >= std::chrono::milliseconds(time_between_key_strokes_ms)) {
                publish_key_event("nothing");
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    endwin();       // restore keyboard settings
}

std::string KeyboardListener::key_to_string(int ch) {
    return std::string(1, ch);
}

void KeyboardListener::publish_key_event(const std::string &key) {
    auto message = std_msgs::msg::String();
    message.data = key;
    publisher->publish(message);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardListener>());
    rclcpp::shutdown();
    return 0;
}
