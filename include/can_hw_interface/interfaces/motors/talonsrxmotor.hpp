#pragma once

#include <memory>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <map>
#include <string>

#include "can_hw_interface/interfaces/genericmotor.hpp"
#include "ctre/Phoenix.h"

namespace robotmotors {

    class TalonSrxMotor : public GenericMotor {
    private:

        //CTRE motor object itself
        TalonSRX* motor;

        //locks out set calls while in follower mode
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

        /*
         * base constructor that calls into the CTRE libs
         * @param id : the CAN ID of the motor as seen by Phoenix Tuner
         */
        TalonSrxMotor(int id);

        /*
         * replaces the contents of type with the type of this
         * @param type : the type of the motor as a string (TalonSRX)
         * @replaces type
         */
        void getType(std::string& type) override;

        /*
         * configures this with a set map of string double pairs
         * @param config : maps string config setting names to double setting values as determined by the method
         * @return : config successful
         */
        bool configure(std::map<std::string, double>& config) override;

        /*
         * configs the PIDF of this with the given parameters in req
         * @param req : requests slot, k_p, k_i, k_d, k_f, i_max, pid_slot
         * @param resp : PIDF config successful
         */
        void configPIDF(const std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Request> req,
                        std::shared_ptr<can_hw_interface::srv::SetPIDFGains::Response> resp) override;

        /*
         * sets this to the given ControlMode PERCENT_OUTPUT, POSITION_CONTROL, VELOCITY_CONTROL, PROFILE_CONTROL, CURRENT_CONTROL
         * with the goal of output as modified by arbOutput
         *
         * @param mode : one of the above control modes
         * @param output : the goal of that control mode
         * @param arbOutput : feedforward component
         */
        void set(ControlMode mode, double output, double arbOutput) override;

        /*
         *
         * @return
         */
        bool getSensorMsg(const can_hw_interface::msg::MotorStatusMsg::SharedPtr msg) override;

        /*
         *
         */
        void setCallback(const can_hw_interface::msg::MotorMsg::SharedPtr msg) override;

        /*
         * destructs the TalonSRX object
         */
        ~TalonSrxMotor();
    };
}