#include "can_hw_interface/interfaces/motors/victorspxmotor.hpp"
#include <iostream>

namespace robotmotors {

    VictorSpxMotor::VictorSpxMotor(int id) {
        motor = new VictorSPX(id);
    }

    void VictorSpxMotor::getType(std::string& type) {
        type = "victorspx";
    }

    bool VictorSpxMotor::configure(std::shared_ptr<std::map<std::string, double>> config) {
        motor->ConfigFactoryDefault();
        //do general config things
        for (std::map<std::string, double>::iterator it = config->begin(); it != config->end(); it++) {
            //physical inversion
            if (it->first == "motor_inverted")
                motor->SetInverted(it->second != 0);
            else if (it->first == "sensor_inverted")
                motor->SetSensorPhase(it->second != 0);

            //master follower TODO (have to figure this one out)
            //TODO look at also reciving device type to follow
            else if (it->first == "follower"){
                motor->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, (int)it->second);
                followerLock = true;
            }

            //feedback settings
            else if (it->first == "feedback_rate")
                motor->SetStatusFramePeriod(Status_2_Feedback0, (int)it->second, 0);
            else if (it->first == "feedback_current")
                feedbackEn.at(2) = it->second != 0;
            else if (it->first == "feedback_voltage")
                feedbackEn.at(3) = it->second != 0;

            //idle mode
            else if (it->first == "neutral_brake")
                motor->SetNeutralMode((it->second != 0 ? NeutralMode::Brake : NeutralMode::Coast));
        }

        //final combined configs
        std::cout << "config complete. last error code is " << motor->GetLastError() << std::endl;

        return motor->GetLastError() == OK;
    }

    void VictorSpxMotor::configPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                                  std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) {
        motor->SelectProfileSlot(req->pid_slot, 0);
        motor->Config_kP(req->pid_slot, req->k_p, 0);
        motor->Config_kI(req->pid_slot, req->k_i, 0);
        motor->Config_IntegralZone(req->pid_slot, req->i_max, 0);
        motor->Config_kD(req->pid_slot, req->k_d, 0);
        motor->Config_kF(req->pid_slot, req->k_f, 0);
        resp->success = motor->GetLastError() == OK;
    }

    /**
     * used to set the output of the motor as well as the output mode
     * if the device is a follower, no call to update the motor is made 
     **/
    void VictorSpxMotor::set(ControlMode mode, double output, double arbOutput) {
        if(followerLock) return;
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output, DemandType::DemandType_ArbitraryFeedForward, arbOutput);       
    }

    bool VictorSpxMotor::registerHostNode(const rclcpp::Node & node){
        
    }


    /*
    // TODO continue for other feedback data needs
    bool VictorSpxMotor::getSensorMsg(const can_msgs::msg::MotorStatusMsg::SharedPtr msg) {
        if(feedbackEn.at(0)) msg->position = motor->GetSelectedSensorPosition();
        if(feedbackEn.at(1)) msg->velocity = motor->GetSelectedSensorVelocity();
        if(feedbackEn.at(3)) msg->voltage = motor->GetBusVoltage();
        return motor->GetLastError() == OK;;
    }

    void VictorSpxMotor::setCallback(const can_msgs::msg::MotorMsg::SharedPtr msg) {
        set(static_cast<ControlMode>(msg->control_mode), msg->demand, msg->arb_feedforward);
    }*/

    VictorSpxMotor::~VictorSpxMotor() {
        delete motor;
    }
}  // namespace robotmotors
