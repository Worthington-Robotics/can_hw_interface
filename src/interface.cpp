#include <tinyxml.h>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#define Phoenix_No_WPI  // remove WPI dependencies
#include "can_hw_interface/interfaces/motors/talonfxmotor.hpp"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class HardwareController : public rclcpp::Node {
private:
    std::map<int, robotmotors::GenericMotor*> motors;
    std::vector<std::string> topics;
    std::vector<rclcpp::Subscription<can_hw_interface::msg::MotorMsg>::SharedPtr> subscriptions;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;
    //rclcpp::TimerBase::SharedPtr timer;

public:
    HardwareController() : Node("can_hw_interface") {
        motors = std::map<int, robotmotors::GenericMotor*>();
        subscriptions = std::vector<rclcpp::Subscription<can_hw_interface::msg::MotorMsg>::SharedPtr>();
        safetySubscrip = create_subscription<std_msgs::msg::Bool>("safety_enable", 10, std::bind(&HardwareController::feedSafety, this, _1));
    }

    void setMotors(std::vector<robotmotors::MotorMap> motorConfig) {
        try {
            //create all motors
            for (robotmotors::MotorMap motorMap : motorConfig) {
                motorMap.topicName = "can_hw_interface/" + motorMap.topicName;

                auto existingElem = std::find(topics.begin(), topics.end(), motorMap.topicName);
                if (existingElem != topics.begin() && existingElem != topics.end()) {
                    throw std::runtime_error("Attempted to create duplicate topic with name " + motorMap.topicName);
                }
                RCLCPP_INFO(this->get_logger(), "creating device on topic %s with ID %d", motorMap.topicName.c_str(), motorMap.canID);
                topics.push_back(motorMap.topicName);

                //create callback
                robotmotors::GenericMotor* motor = new robotmotors::TalonFxMotor(motorMap.canID);
                RCLCPP_INFO(this->get_logger(), "created device");

                //configuration phase
                if (!motor->configure(motorMap.config)) {
                    throw std::runtime_error("Device returned non-ok error code after configuration");
                }

                //push the callback and subscription onto their vectors
                subscriptions.push_back(this->create_subscription<can_hw_interface::msg::MotorMsg>(motorMap.topicName, 10,
                                                                                                   std::bind(&robotmotors::GenericMotor::setCallback, motor, _1)));

                this->motors[motorMap.canID] = motor;

                RCLCPP_INFO(this->get_logger(), "registered motor");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind motor\nCause: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind motor\nCause Unknown");
        }
    }

    void feedSafety(std::shared_ptr<std_msgs::msg::Bool> msg) {
        if (msg->data) ctre::phoenix::unmanaged::FeedEnable(100);
    }

    void neutralMotors() {
        for (auto it = this->motors.begin(); it != this->motors.end(); it++) {
            it->second->set(robotmotors::PERCENT_OUTPUT, 0, 0);
        }
    }

    ~HardwareController() {
        neutralMotors();
        for (size_t i = 0; i < motors.size(); i++) {
            delete motors.at(i);
        }
        motors.clear();
    }
};

int main(int argc, char** argv) {
    //init ros node
    rclcpp::init(argc, argv);
    std::shared_ptr<HardwareController> rosNode = std::make_shared<HardwareController>();
    RCLCPP_INFO(rosNode->get_logger(), "hardware interface node starting");

    //init canbus
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    std::string xmlDoc = "";

    /* make some talons for drive train */
    std::vector<robotmotors::MotorMap> test = std::vector<robotmotors::MotorMap>();

    TiXmlDocument * doc = new TiXmlDocument();
    doc->Parse(xmlDoc.c_str(), 0, TIXML_ENCODING_UNKNOWN);

    std::map<std::string, double> leftConfig = std::map<std::string, double>();
    leftConfig["motor_inverted"] = 1;
    leftConfig["curr_limit_enable"] = 0;
    leftConfig["feedback_rate"] = 5;
    leftConfig["neutral_brake"] = 1;
    leftConfig["vcomp_voltage"] = 11.0;
    robotmotors::MotorMap leftMap = robotmotors::MotorMap();
    leftMap.canID = 1;
    leftMap.topicName = "left";
    leftMap.config = leftConfig;
    test.push_back(leftMap);

    std::map<std::string, double> leftFollower = std::map<std::string, double>();
    leftFollower["motor_inverted"] = 1;
    leftFollower["curr_limit_enable"] = 0;
    leftFollower["feedback_rate"] = 5;
    leftFollower["neutral_brake"] = 1;
    leftFollower["vcomp_voltage"] = 11.0;
    leftFollower["follower"] = 1;
    robotmotors::MotorMap leftFollowerMap = robotmotors::MotorMap();
    leftFollowerMap.canID = 2;
    leftFollowerMap.topicName = "left_follower";
    leftFollowerMap.config = leftFollower;
    test.push_back(leftFollowerMap);

    std::map<std::string, double> rightConfig = std::map<std::string, double>();
    rightConfig["motor_inverted"] = 0;
    rightConfig["curr_limit_enable"] = 0;
    rightConfig["feedback_rate"] = 5;
    rightConfig["neutral_brake"] = 1;
    rightConfig["vcomp_voltage"] = 11.0;
    robotmotors::MotorMap rightMap = robotmotors::MotorMap();
    rightMap.canID = 3;
    rightMap.topicName = "right";
    rightMap.config = rightConfig;
    test.push_back(rightMap);

    std::map<std::string, double> rightFollower = std::map<std::string, double>();
    rightFollower["motor_inverted"] = 0;
    rightFollower["curr_limit_enable"] = 0;
    rightFollower["feedback_rate"] = 5;
    rightFollower["neutral_brake"] = 1;
    rightFollower["vcomp_voltage"] = 11.0;
    rightFollower["follower"] = 3;
    robotmotors::MotorMap rightFollowerMap = robotmotors::MotorMap();
    rightFollowerMap.canID = 4;
    rightFollowerMap.topicName = "right_follower";
    rightFollowerMap.config = rightFollower;
    test.push_back(rightFollowerMap);

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