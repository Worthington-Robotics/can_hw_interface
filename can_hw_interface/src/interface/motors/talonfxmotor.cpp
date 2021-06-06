#include "can_hw_interface/interfaces/motors/talonfxmotor.hpp"
#include <iostream>


namespace robotmotors {

    TalonFxMotor::TalonFxMotor(int id) {
        motor = new TalonFX(id);
    }

    void TalonFxMotor::getType(std::string& type) {
        type = "talonfx";
    }

    bool TalonFxMotor::configure(std::shared_ptr<std::map<std::string, double>> config) {
        motor->ConfigFactoryDefault();
        motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
        motor->SelectProfileSlot(0, 0);
        double currentLimitVal = 0, currentLimitTime = 0, currentLimitTrigger = 0;
        bool currentLimEnable = false;
        feedbackEn.resize(4);
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
            else if (it->first == "feedback_position")
                feedbackEn.at(0) = it->second != 0;
            else if (it->first == "feedback_velocity")
                feedbackEn.at(1) = it->second != 0;
            else if (it->first == "feedback_current")
                feedbackEn[2] = it->second != 0;
            else if (it->first == "feedback_voltage")
                feedbackEn[3] = it->second != 0;

            //idle mode
            else if (it->first == "neutral_brake")
                motor->SetNeutralMode((it->second != 0 ? NeutralMode::Brake : NeutralMode::Coast));

            //pid settings
            else if (it->first == "vcomp_voltage") {
                motor->ConfigVoltageCompSaturation(it->second);
                motor->EnableVoltageCompensation(true);
            } else if (it->first == "pid_kp")
                motor->Config_kP(0, it->second, 0);
            else if (it->first == "pid_ki")
                motor->Config_kI(0, it->second, 0);
            else if (it->first == "pid_izone")
                motor->Config_IntegralZone(0, it->second, 0);
            else if (it->first == "pid_kd")
                motor->Config_kD(0, it->second, 0);
            else if (it->first == "pid_kf")
                motor->Config_kF(0, it->second, 0);

            //current clamping
            else if (it->first == "curr_limit_trig")
                currentLimitTrigger = it->second;
            else if (it->first == "curr_limit_val")
                currentLimitVal = it->second;
            else if (it->first == "curr_limit_time")
                currentLimitTime = it->second;
            else if (it->first == "curr_limit_enable")
                currentLimEnable = it->second != 0;
        }

        //final combined configs
        motor->ConfigStatorCurrentLimit({currentLimEnable, currentLimitVal, currentLimitTrigger, currentLimitTime});

        //std::cout << "config complete. last error code is " << motor->GetLastError() << std::endl;

        return motor->GetLastError() == OK;
    }

    void TalonFxMotor::configPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
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
    void TalonFxMotor::set(ControlMode mode, double output, double arbOutput) {
        if(followerLock) return;
        if(mode == POSITION_CONTROL){
            motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, output, DemandType::DemandType_ArbitraryFeedForward, arbOutput);
        } else if(mode == VELOCITY_CONTROL){
            motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, output, DemandType::DemandType_ArbitraryFeedForward, arbOutput);
        } else if(mode == CURRENT_CONTROL){
            motor->Set(ctre::phoenix::motorcontrol::ControlMode::Current, output, DemandType::DemandType_ArbitraryFeedForward, arbOutput);
        } else if(mode == PROFILE_CONTROL){
            motor->Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, output, DemandType::DemandType_ArbitraryFeedForward, arbOutput);
        } else {
            motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output, DemandType::DemandType_ArbitraryFeedForward, arbOutput);
        }
        
    }

    bool TalonFxMotor::registerHostNode(const rclcpp::Node & node){
        
    }

    // TODO continue for other feedback data needs
    /*bool TalonFxMotor::getSensorMsg(const can_msgs::msg::MotorStatusMsg::SharedPtr msg) {
        if(feedbackEn.at(0)) msg->position = motor->GetSelectedSensorPosition();
        if(feedbackEn.at(1)) msg->velocity = motor->GetSelectedSensorVelocity();
        if(feedbackEn.at(2)) msg->current = motor->GetStatorCurrent();
        if(feedbackEn.at(3)) msg->voltage = motor->GetBusVoltage();
        return motor->GetLastError() == OK;;
    }

    void TalonFxMotor::setCallback(const can_msgs::msg::MotorMsg::SharedPtr msg) {
        set(static_cast<ControlMode>(msg->control_mode), msg->demand, msg->arb_feedforward);
    }*/

    TalonFxMotor::~TalonFxMotor() {
        delete motor;
    }
}  // namespace robotmotors
