#pragma once

#include <map>
#include <string>

#include "can_msgs/msg/motor_msg.hpp"
#include "can_msgs/msg/motor_status_msg.hpp"
#include "can_msgs/srv/set_pidf_gains.hpp"
#include "genericsensor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotmotors {

    //List of valid & registered motor types
    enum MotorType {
        TALONFX,
        TALONSRX,
        VICTORSPX,
        UNKNOWN
    };

    /**
     * gets the internal motor type from its string representation
     * @param motorType the string representation of the motor type
     * @param type the MotorType variable to store the result in
     * @returns true if there were no errors during the parse operation
     **/
    bool getMotorType(const std::string& motorType, MotorType& type);

    /**
     * gets the enum ordinal for motor type from its string value 
     * @param motorType the string representation of the motor type
     * @param ordinal the integer to store the resulting value
     * @returns true if there was no errors parsing
     **/
    bool getMotorInt(std::string& motorType, int& ordinal);

    /**
     * gets the enum ordinal for motor type from its enum value 
     * @param motorType the enum representation of the motor type
     * @param ordinal the integer to store the resulting value
     * @returns true if there was no errors parsing
     **/
    bool getMotorInt(MotorType motorType, int& ordinal);

    enum ControlMode {
        PERCENT_OUTPUT,
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        PROFILE_CONTROL,
        CURRENT_CONTROL,
    };

    struct MotorMap {
        std::string topicName;
        MotorType motorType;
        int canID;
        std::shared_ptr<std::map<std::string, double>> config;
    };

    class GenericMotor {

    protected:
        //string topic for the motor to use as a base
        std::shared_ptr<std::string> topic;

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
        virtual bool configure(rclcpp::Node & node, rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> opts, std::string & topicStr, std::shared_ptr<std::map<std::string, double>> config) = 0;

        /**
         * ROS service for configuring PIDF values dynamically while the controller is operating
         * This should be a stable and deterministic method of configuration
         **/
        virtual void configPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                                std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) = 0;

        /**
         * generic set function for internal calls
         * should set the motor controller demand as well as control mode and an arbitrary input value
         **/
        virtual void set(ControlMode mode, double output, double arbInput) = 0;

        /**
         * callback for ROS messages to set the state of the motor
         * probably should call the generic form of the set function with the ROS message values
         **/
        virtual void setCallback(const can_msgs::msg::MotorMsg::SharedPtr msg) = 0;

        /*
         * function call to allow this motor to publish all related sensor data to a ros topic
         */
        virtual void publishNewSensorData() = 0;


        virtual ~GenericMotor() = default;
    };
}
