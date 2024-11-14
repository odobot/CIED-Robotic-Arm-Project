#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

class KeyboardTestNode : public rclcpp::Node
{
public:
    KeyboardTestNode() : Node("keyboard_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Keyboard test node started. Press keys and see if they are logged. Press 'Q' to quit.");
    }

    void run()
    {
        char c;
        struct termios oldt, newt;
        // Set terminal to raw mode to capture individual key presses
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok())
        {
            c = getchar();
            RCLCPP_INFO(this->get_logger(), "Key pressed: 0x%02X", c);

            if (c == 'q' || c == 'Q')
            {
                RCLCPP_INFO(this->get_logger(), "Exiting keyboard test node.");
                break;
            }
        }

        // Restore the original terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTestNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
