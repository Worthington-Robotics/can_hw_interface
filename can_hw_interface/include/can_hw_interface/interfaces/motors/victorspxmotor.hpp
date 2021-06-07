#pragma once

#include <memory>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <map>
#include <string>

#include "can_hw_interface/interfaces/genericmotor.hpp"
#include "ctre/Phoenix.h"

namespace robotmotors {

    class VictorSpxMotor : public GenericMotor {
    private:
        VictorSPX* motor;

        bool followerLock = false;

        //stores what feedback components should be collected
        //0 -- position
        //1 -- velocity
        //2 -- current
        //3 -- voltage
        //4 -- lower limit
        //5 -- upper limit
        std::vector<bool> feedbackEn;

        // ros2 subscription pointer for listening on output demand topic
        rclcpp::Subscription<can_msgs::msg::MotorMsg>::SharedPtr demands;

        //ros2 timer to handle publishing data from sensors
        rclcpp::TimerBase::SharedPtr updateTimer;

        // ros2 publisher for publishing data from sensor inputs
        rclcpp::Publisher<can_msgs::msg::MotorStatusMsg>::SharedPtr publisher;

    public:
        VictorSpxMotor(int id);

        void getType(std::string& type) override;

        bool configure(rclcpp::Node & node, std::string & topicStr, std::shared_ptr<std::map<std::string, double>> config) override;

        void configPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                        std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) override;

        void set(ControlMode mode, double output, double arbOutput) override;

        /*
         *
         */
        void publishNewSensorData();

        /*
         * sets this using the data from the msg file
         */
        void setCallback(const can_msgs::msg::MotorMsg::SharedPtr msg) override;

        ~VictorSpxMotor();
    };
}