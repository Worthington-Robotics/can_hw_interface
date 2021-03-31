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

    public:
        VictorSpxMotor(int id);

        void getType(std::string& type) override;
        bool configure(std::shared_ptr<std::map<std::string, double>> config) override;
        void configPIDF(const std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Request> req,
                        std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Response> resp) override;
        void set(ControlMode mode, double output, double arbOutput) override;
        bool getSensorMsg(const can_hw_interface::msg::MotorStatusMsg::SharedPtr msg) override;
        void setCallback(const can_hw_interface::msg::MotorMsg::SharedPtr msg) override;

        ~VictorSpxMotor();
    };
}