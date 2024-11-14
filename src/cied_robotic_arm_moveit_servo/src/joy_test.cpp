#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class GamepadController : public rclcpp::Node {
public:
    GamepadController() : Node("gamepad_controller"), mode_("joint_control") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&GamepadController::joyCallback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_server/delta_twist_cmds", 10);

        
        joint_cmds_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/servo_server/delta_joint_cmds", 10);

        RCLCPP_INFO(this->get_logger(), "Gamepad controller node initialized.");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Mode switching logic
        if (msg->buttons[0] == 1) { // A button
            mode_ = "joint_control";
            RCLCPP_INFO(this->get_logger(), "Switched to Joint Control Mode");
        } else if (msg->buttons[1] == 1) { // B button
            mode_ = "cartesian_control";
            RCLCPP_INFO(this->get_logger(), "Switched to Cartesian Control Mode");
        }

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->get_clock()->now();

        // Cartesian Control Mode
        if (mode_ == "cartesian_control") {
            twist_msg.twist.linear.x = msg->axes[0];  // Left stick X for x movement
            twist_msg.twist.linear.y = msg->axes[1];  // Left stick Y for y movement
            twist_msg.twist.linear.z = (msg->buttons[3] == 1) ? 0.5 : (msg->buttons[4] == 1 ? -0.5 : 0.0);  // Z-axis control with X/Y buttons
            twist_msg.twist.angular.z = msg->axes[2];  // Right stick X for rotation

            twist_pub_->publish(twist_msg);
        }

        // Future expansion for joint control mode
        if (mode_ == "joint_control") {
            control_msgs::msg::JointJog joint_cmds;
            joint_cmds.header.stamp = this->get_clock()->now();

            // // Example mapping of joystick axes to joint commands
            // joint_cmds.velocities[0] = msg->axes[4];  // Left trigger for joint 1
            // joint_cmds.velocities[1] = msg->axes[5];  // Right trigger for joint 2
            // joint_cmds.velocities[2] = msg->axes[2];  // Left stick X for joint 3
            // joint_cmds.velocities[3] = msg->axes[3];  // Left stick Y for joint 4
            // joint_cmds.velocities[4] = msg->axes[0];  // Right stick X for joint 5
        }
    }

    std::string mode_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmds_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamepadController>());
    rclcpp::shutdown();
    return 0;
}
