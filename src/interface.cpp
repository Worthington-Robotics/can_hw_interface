#include <tinyxml.h>

#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#define Phoenix_No_WPI  // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

using std::placeholders::_1;
using namespace std::chrono_literals;

struct MotorMap {
    std::string topicName;
    int canID;
};

class MotorCallback {
public:
    MotorCallback(int canID, rclcpp::Node* node) {
        rosNode = node;
        motor = new TalonFX(canID);
        setNeutral();
        std::cout << motor->GetLastError() << std::endl;
        if (motor->GetLastError() != ctre::phoenix::ErrorCode::OK) {
            throw std::runtime_error("failed to initialize device with reason" + std::to_string(motor->GetLastError()));
        }
    }

    void callback(const std_msgs::msg::Float64::SharedPtr msg) {
        //RCLCPP_INFO(rosNode->get_logger(), "got callback with data [ %f ] to motor ID: %i", (double)msg->data, motor->GetDeviceID());
        motor->Set(ControlMode::PercentOutput, (double)msg->data);
    }

    void setNeutral() {
        motor->Set(ControlMode::PercentOutput, 0);
    }

    ~MotorCallback(){
        delete motor;
    }

private:
    std::string topicName;
    TalonFX * motor;
    rclcpp::Node * rosNode;
};

class MotorSubscriber : public rclcpp::Node {
public:
    MotorSubscriber() : Node("can_hw_interface") {
        subscriptions = std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>();
        motors = std::vector<MotorCallback*>();
        timer = this->create_wall_timer(50ms, std::bind(&MotorSubscriber::feedSafety, this));
    }

    void setMotors(std::vector<MotorMap> motors) {
        for (MotorMap motor : motors) {
            try {
                motor.topicName = "can_hw_interface/" + motor.topicName;
                RCLCPP_INFO(this->get_logger(), "creating device on topic %s with ID %d", motor.topicName.c_str(), motor.canID );
                
                //create callback
                MotorCallback*  cb = new MotorCallback(motor.canID, this);
                //RCLCPP_INFO_ONCE(this->get_logger(), "created device");

                //push the callback and subscription onto their vectors
                subscriptions.push_back(this->create_subscription<std_msgs::msg::Float64>(motor.topicName, 10, std::bind(&MotorCallback::callback, cb, _1)));

                //RCLCPP_INFO_ONCE(this->get_logger(), "subscribed topic for device");

                this->motors.push_back(cb);

                //RCLCPP_INFO_ONCE(this->get_logger(), "registered motor");
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind motor to ID %f\nCause: %s", motor.canID, e.what());
            }
        }
    }

    void feedSafety(){
        ctre::phoenix::unmanaged::FeedEnable(100);
    }

    void neutralMotors() {
        for (MotorCallback* motor : motors) {
            motor->setNeutral();
        }
    }

    ~MotorSubscriber(){
        neutralMotors();
        for (size_t i = 0; i < motors.size(); i++) {
            delete motors.at(i);
        }
        motors.clear();
    }

private:
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscriptions;
    std::vector<MotorCallback*> motors;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
    //init ros node
    rclcpp::init(argc, argv);
    std::shared_ptr<MotorSubscriber> rosNode = std::make_shared<MotorSubscriber>();
    RCLCPP_INFO(rosNode->get_logger(), "hardware interface node starting");

    //init canbus
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    /* make some talons for drive train */
    std::vector<MotorMap> test = std::vector<MotorMap>();
    test.push_back({"left", 1});
    test.push_back({"right", 3});
    rosNode->setMotors(test);

    //set all motors to neutral
    rosNode->neutralMotors();

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface node loaded using can interface %s", interface.c_str());

    // serve the callbacks
    rclcpp::spin(rosNode);

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface shutting down");

    //set all motors to neutral
    rosNode->neutralMotors();

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface shut down complete");

    rclcpp::shutdown();

    return 0;
}