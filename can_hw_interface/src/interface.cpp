#include <tinyxml.h>

#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#define Phoenix_No_WPI  // remove WPI dependencies
#include "can_hw_interface/interfaces/motorparser.hpp"
#include "can_hw_interface/interfaces/motors/talonfxmotor.hpp"
#include "can_hw_interface/interfaces/motors/talonsrxmotor.hpp"
#include "can_hw_interface/interfaces/motors/victorspxmotor.hpp"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define SAFETY_TIMEOUT_MS 100

using std::placeholders::_1;
using namespace std::chrono_literals;

class HardwareController : public rclcpp::Node {
private:
    //list of all motors registered to the system
    std::map<int, std::shared_ptr<robotmotors::GenericMotor>> motors;

    //list of all subscribed topics
    std::vector<std::string> topics;

    //safety enable subscription that allows motors to be active
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safetySubscrip;

    rclcpp::TimerBase::SharedPtr lowRate, midRate, highRate;

    std::vector<std::shared_ptr<robotmotors::GenericMotor>> lowRateMotors, midRateMotors, highRateMotors;

    bool initComplete = false;

    rclcpp::CallbackGroup::SharedPtr subs, pubs;
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subsOpt;

public:
    HardwareController() : Node("can_hw_interface") {
        motors = std::map<int, std::shared_ptr<robotmotors::GenericMotor>>();

        // Register publisher ans subscriber callback groups
        subs = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        pubs = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        subsOpt = rclcpp::SubscriptionOptions();
        subsOpt.callback_group = subs;

        safetySubscrip = create_subscription<std_msgs::msg::Bool>("safety_enable", 10, std::bind(&HardwareController::feedSafety, this, _1), subsOpt);

        // Create publish timers at different rates
        lowRate = create_wall_timer(100ms, std::bind(&HardwareController::lowRateCallback, this), pubs);
        midRate = create_wall_timer(20ms, std::bind(&HardwareController::midRateCallback, this), pubs);
        highRate = create_wall_timer(10ms, std::bind(&HardwareController::highRateCallback, this), pubs);
    }

    void setMotors(std::shared_ptr<std::vector<robotmotors::MotorMap>> motorConfig) {
        try {
            // Create all motors
            for (auto it = motorConfig->begin(); it != motorConfig->end(); it++) {
                it->topicName = "can_hw_interface/" + it->topicName;

                auto existingElem = std::find(topics.begin(), topics.end(), it->topicName);
                if (existingElem != topics.begin() && existingElem != topics.end()) {
                    throw std::runtime_error("Attempted to create duplicate topic with name " + it->topicName);
                }
                RCLCPP_INFO(this->get_logger(), "creating device on topic %s with ID %d", it->topicName.c_str(), it->canID);
                topics.push_back(it->topicName);

                // Create callback
                std::shared_ptr<robotmotors::GenericMotor> motor;
                switch (it->motorType) {
                case robotmotors::VICTORSPX:
                    motor = std::make_shared<robotmotors::VictorSpxMotor>(it->canID);
                    break;
                case robotmotors::TALONFX:
                    motor = std::make_shared<robotmotors::TalonFxMotor>(it->canID);
                    break;
                case robotmotors::TALONSRX:
                    motor = std::make_shared<robotmotors::TalonSrxMotor>(it->canID);
                    break;
                default:
                    throw std::runtime_error("No valid motor type defined. Got: " + it->motorType);
                }

                // Configuration phase
                if (!motor->configure(*this, subsOpt, it->topicName, it->config)) {
                    throw std::runtime_error("Device returned non-ok error code after configuration");
                }

                // Make sure feedback rate is present
                if (it->config->find("feedback_rate") != it->config->end()) {
                    // Add to high rate list
                    if (it->config->at("feedback_rate") < 11.0) this->highRateMotors.push_back(motor);

                    // Add to mid rate list
                    else if (it->config->at("feedback_rate") < 21.0)
                        this->midRateMotors.push_back(motor);

                    // Add to low rate list
                    else
                        this->lowRateMotors.push_back(motor);
                }

                this->motors[it->canID] = motor;
            }

            initComplete = true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind motor\nCause: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind motor\nCause Unknown");
        }
    }

    void feedSafety(std::shared_ptr<std_msgs::msg::Bool> msg) {
        if (msg->data) ctre::phoenix::unmanaged::FeedEnable(SAFETY_TIMEOUT_MS);
    }

    void neutralMotors() {
        for (auto it = this->motors.begin(); it != this->motors.end(); it++) {
            it->second->set(robotmotors::PERCENT_OUTPUT, 0, 0);
        }
    }

    bool hasInit() {
        return initComplete;
    }

    void lowRateCallback() {
        for (auto& motor : lowRateMotors) {
            motor->publishNewSensorData();
        }
    }

    void midRateCallback() {
        for (auto& motor : midRateMotors) {
            motor->publishNewSensorData();
        }
    }

    void highRateCallback() {
        for (auto& motor : highRateMotors) {
            motor->publishNewSensorData();
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

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<HardwareController> rosNode = std::make_shared<HardwareController>();
    executor.add_node(rosNode);
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

            if (!rosNode->hasInit()) {
                throw std::runtime_error("Device initalization step failed. See above logs for more details.\nExiting");
            }

            //set all motors to neutral
            rosNode->neutralMotors();

            RCLCPP_INFO(rosNode->get_logger(), "hardware interface node loaded using can interface %s", interface.c_str());

            // serve the callbacks
            executor.spin();

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