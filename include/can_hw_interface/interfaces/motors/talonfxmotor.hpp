#pragma once

#include <memory>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <map>
#include <string>

#include "can_hw_interface/interfaces/genericmotor.hpp"
#include "ctre/Phoenix.h"

namespace robotmotors {

    class TalonFxMotor : public GenericMotor {
    private:
        TalonFX* motor;

    public:
        TalonFxMotor(int id);

        void getType(std::string& type) override;
        bool configure(std::map<std::string, double>& config) override;
        void configPIDF(const std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Request> req,
                        std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Response> resp) override;
        void set(ControlMode mode, double output, double arbOutput) override;
        bool getSensor(robotsensors::GenericSensor& sens) override;
        void setCallback(const can_hw_interface::msg::MotorMsg::SharedPtr msg) override;

        ~TalonFxMotor();
    };
}