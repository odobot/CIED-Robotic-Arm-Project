#ifndef CIED_ROBOTIC_ARM_HARDWARE_HPP
#define CIED_ROBOTIC_ARM_HARDWARE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <libserial/SerialPort.h>

namespace cied_robotic_arm_hardware
{
    class CIEDRoboticArm : public hardware_interface::SystemInterface{
        public:
            CIEDRoboticArm();

            virtual ~CIEDRoboticArm();

            // Lifecycle Node Interface Callbacks
            virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            // Hardware Interface  
            virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
           LibSerial::SerialPort serial_port_;
           std::string serial_port_name_;

           std::vector<double> position_commands;
           std::vector<double> prev_position_commands;
           std::vector<double> position_states;
    };
}

#endif