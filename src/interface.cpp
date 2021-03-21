#include <tinyxml.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/* make some talons for drive train */
TalonFX talLeft(1);
TalonFX talRght(0);

int main(int argc, char **argv) {
    //init ros node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> rosNode = rclcpp::Node::make_shared("hw_interface");
    RCLCPP_INFO(rosNode->get_logger(), "hardware interface node starting");

    //init canbus
    std::string interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface node loaded using can interface %s", interface);

    talLeft.Set(ControlMode::PercentOutput, 0.0);
    talRght.Set(ControlMode::PercentOutput, 0.0);

    // Zzzzzz.
    rclcpp::spin(rosNode);

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface shutting down");

    rclcpp::shutdown();

    return 0;
}