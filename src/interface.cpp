#include <tinyxml.h>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <filesystem>

#define Phoenix_No_WPI  // remove WPI dependencies
#include "can_hw_interface/interfaces/motorparser.hpp"
#include "can_hw_interface/interfaces/motors/talonfxmotor.hpp"
#include "can_hw_interface/interfaces/motors/victorspxmotor.hpp"
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

    //list of all motors registered to the system
    std::map<int, std::shared_ptr<robotmotors::GenericMotor>> motors;

    //list of all subscribed topics
    std::vector<std::string> topics;

    //subscriptions for all motor inputs
    std::vector<rclcpp::Subscription<can_hw_interface::msg::MotorMsg>::SharedPtr> subscriptions;

    //safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    //contains 3 timers used to execute update tasks
    //index 0: high rate (10ms)
    //index 1: medium rate (20ms)
    //index 2: low rate (100ms)
    std::vector<rclcpp::TimerBase::SharedPtr> timers;

public:
    HardwareController() : Node("can_hw_interface") {
        motors = std::map<int, std::shared_ptr<robotmotors::GenericMotor>>();
        subscriptions = std::vector<rclcpp::Subscription<can_hw_interface::msg::MotorMsg>::SharedPtr>();
        safetySubscrip = create_subscription<std_msgs::msg::Bool>("safety_enable", 10, std::bind(&HardwareController::feedSafety, this, _1));
    }

    void setMotors(std::shared_ptr<std::vector<robotmotors::MotorMap>> motorConfig) {
        try {
            //create all motors
            for (auto it = motorConfig->begin(); it != motorConfig->end(); it++) {
                it->topicName = "can_hw_interface/" + it->topicName;

                auto existingElem = std::find(topics.begin(), topics.end(), it->topicName);
                if (existingElem != topics.begin() && existingElem != topics.end()) {
                    throw std::runtime_error("Attempted to create duplicate topic with name " + it->topicName);
                }
                RCLCPP_INFO(this->get_logger(), "creating device on topic %s with ID %d", it->topicName.c_str(), it->canID);
                topics.push_back(it->topicName);

                //create callback
                std::shared_ptr<robotmotors::GenericMotor> motor;
                switch (it->motorType) {
                    case robotmotors::VICTORSPX:
                        motor = std::make_shared<robotmotors::VictorSpxMotor>(it->canID);
                        break;
                    case robotmotors::TALONFX:
                        motor = std::make_shared<robotmotors::TalonFxMotor>(it->canID);
                        break;
                    case robotmotors::TALONSRX:
                    default:
                        throw std::runtime_error("No valid motor type defined. Got: " + it->motorType);
                }

                //configuration phase
                if (!motor->configure(it->config)) {
                    throw std::runtime_error("Device returned non-ok error code after configuration");
                }

                //push the callback and subscription onto their vectors
                subscriptions.push_back(this->create_subscription<can_hw_interface::msg::MotorMsg>(it->topicName, 10,
                                                                                                   std::bind(&robotmotors::GenericMotor::setCallback, motor, _1)));

                this->motors[it->canID] = motor;
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

    //TODO fix error here
    ~HardwareController() {
        neutralMotors();
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

    // Load the xml file
    std::filesystem::path config = std::filesystem::current_path() / "config.xml";
    TiXmlDocument* doc = new TiXmlDocument(config.c_str());
    if (!doc->LoadFile()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Error parsing XML config %s\n %s", config.c_str(), doc->ErrorDesc());

    } else {
        try {
            std::shared_ptr<std::vector<robotmotors::MotorMap>> motors = robotmotors::createMotorMap(doc);
            RCLCPP_INFO(rosNode->get_logger(), "Recieved config for %d motor(s)", motors->size());
            rosNode->setMotors(motors);

            //set all motors to neutral
            rosNode->neutralMotors();

            RCLCPP_INFO(rosNode->get_logger(), "hardware interface node loaded using can interface %s", interface.c_str());

            // serve the callbacks
            rclcpp::spin(rosNode);

            RCLCPP_INFO(rosNode->get_logger(), "hardware interface shutting down");

            //set all motors to neutral
            rosNode->neutralMotors();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rosNode->get_logger(), "Node failed\nCause: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(rosNode->get_logger(), "Node failed\nCause Unknown");
        }
    }

    delete doc;

    RCLCPP_INFO(rosNode->get_logger(), "hardware interface shut down complete");

    rclcpp::shutdown();

    return 0;
}