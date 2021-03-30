#pragma once

#include <map>
#include <string>

#include "can_hw_interface/msg/motor_msg.hpp"
#include "can_hw_interface/msg/motor_status_msg.hpp"
#include "can_hw_interface/srv/set_pidf_gains.hpp"
#include "genericsensor.hpp"

namespace robotmotors {

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        PROFILE_CONTROL,
        CURRENT_CONTROL,
    };

    class GenericMotor {
    public:
        /**
         * gets a string alias for the motor controller type used internally 
         **/
        virtual void getType(std::string& type) = 0;

        /**
         * configures the motor based on the presence of data in a map
         * the device is expected to be defaulted, then configured as expected
         * returns true if the device was configured without errors
         **/
        virtual bool configure(std::map<std::string, double> & config) = 0;

        /**
         * ROS service for configuring PIDF values dynamically while the controller is operating
         * This should be a stable and deterministic method of configuration
         **/
        virtual void configPIDF(const std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Request> req,
                                std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Response> resp) = 0;

        /**
         * generic set function for internal calls
         * should set the motor controller demand as well as control mode and an arbitrary input value
         **/
        virtual void set(ControlMode mode, double output, double arbInput) = 0;

        /**
         * gets the status reporting message to be published containing feedback data
         * returns true if the device gave all values ok
         **/ 
        virtual bool getSensorMsg(const can_hw_interface::msg::MotorStatusMsg::SharedPtr msg) = 0;

        /**
         * callback for ROS messages to set the state of the motor
         * probably should call the generic form of the set function with the ROS message values
         **/ 
        virtual void setCallback(const can_hw_interface::msg::MotorMsg::SharedPtr msg) = 0;

        virtual ~GenericMotor() = default;
    };
}
