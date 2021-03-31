#include "can_hw_interface/interfaces/motorparser.hpp"

namespace robotmotors {

    bool getValue(TiXmlElement* elem, const std::string& childName, std::string& value);
    bool stob(std::string s);

    std::shared_ptr<std::vector<robotmotors::MotorMap>> createMotorMap(TiXmlDocument* doc) {
        std::shared_ptr<std::vector<robotmotors::MotorMap>> motorMaps = std::make_shared<std::vector<robotmotors::MotorMap>>();

        // grab the parent motor XML element and make sure it exists
        TiXmlElement* hardware = doc->FirstChildElement("hardware");
        if (!hardware) throw std::runtime_error("XML doc is missing root hardware element");
        TiXmlElement* motors = hardware->FirstChildElement("motors");
        if (!motors) throw std::runtime_error("XML doc is missing motors element. The motors element should be defined even if there are no motors being created");

        //iterate through all declared motors
        for (TiXmlElement* motor = motors->FirstChildElement("motor"); motor != nullptr; motor = motor->NextSiblingElement("motor")) {
            //initialization work
            MotorMap motorMap = MotorMap();
            bool fail[3] = {false, false, false}; std::string temp = "";
            std::shared_ptr<std::map<std::string, double>> config = std::make_shared<std::map<std::string, double>>();

            //make sure required elements exist
            fail[0] = getValue(motor, "topic", motorMap.topicName);
            fail[1] = getValue(motor, "canid", temp);
            fail[2] = getValue(motor, "type", motorMap.motorType);
            if (fail[0] || fail[1] || fail[2]) throw std::runtime_error(std::string("Motor definintion is missing required field(s): ")+ 
            (fail[0]? "topic " : "") + (fail[1]? "canid " : "") + (fail[2]? "type " : ""));
            motorMap.canID = std::stoi(temp);

            //work through config elements
            if (getValue(motor, "motor_inverted", temp)) config->insert(std::pair<std::string, double>("motor_inverted", stob(temp)? 1.0 : 0.0));
            if (getValue(motor, "sensor_inverted", temp)) config->insert(std::pair<std::string, double>("sensor_inverted", stob(temp)? 1.0 : 0.0));
            if (getValue(motor, "vcomp_voltage", temp)) config->insert(std::pair<std::string, double>("vcomp_voltage", stod(temp)));

            //handle feedback element
            TiXmlElement* feedback = motor->FirstChildElement("feedback");
            if (feedback) {
                if (getValue(motor, "rate", temp)) config->insert(std::pair<std::string, double>("feedback_rate", stob(temp)? 1.0 : 0.0));
                if (getValue(motor, "position", temp)) config->insert(std::pair<std::string, double>("feedback_position", std::stod(temp)));
                if (getValue(motor, "velocity", temp)) config->insert(std::pair<std::string, double>("feedback_velocity", std::stod(temp)));
                if (getValue(motor, "current", temp)) config->insert(std::pair<std::string, double>("feedback_current", std::stod(temp)));
                if (getValue(motor, "voltage", temp)) config->insert(std::pair<std::string, double>("feedback_voltage", std::stod(temp)));
            }

            //handle current_limit element
            TiXmlElement* currentLimit = motor->FirstChildElement("current_limit");
            if (currentLimit) {
                if (getValue(motor, "enable", temp)) config->insert(std::pair<std::string, double>("current_limit_enable", stob(temp)? 1.0 : 0.0));
                if (getValue(motor, "trigger", temp)) config->insert(std::pair<std::string, double>("current_limit_trig", std::stod(temp)));
                if (getValue(motor, "val", temp)) config->insert(std::pair<std::string, double>("current_limit_val", std::stod(temp)));
                if (getValue(motor, "time", temp)) config->insert(std::pair<std::string, double>("current_limit_time", std::stod(temp)));
            }

            //handle pid_config
            TiXmlElement* pidConfig = motor->FirstChildElement("pid_config");
            if (pidConfig) {
                if (getValue(motor, "kp", temp)) config->insert(std::pair<std::string, double>("pid_kp", std::stod(temp)));
                if (getValue(motor, "ki", temp)) config->insert(std::pair<std::string, double>("pid_ki", std::stod(temp)));
                if (getValue(motor, "kd", temp)) config->insert(std::pair<std::string, double>("pid_kd", std::stod(temp)));
                if (getValue(motor, "kf", temp)) config->insert(std::pair<std::string, double>("pid_kf", std::stod(temp)));
                if (getValue(motor, "izone", temp)) config->insert(std::pair<std::string, double>("pid_izone", std::stod(temp)));
            }

            motorMap.config = config;
            motorMaps->push_back(motorMap);
        }

        return motorMaps;
    }

    /**
     * function to pull a child element from a parent element as text
     * @param elem the parent XML element
     * @param childName the name of the XML child element to find
     * @param value the resulting value of the child element.
     * @return bool true if the element was not found or false if found
     **/
    bool getValue(TiXmlElement* elem, const std::string& childName, std::string& value) {
        //std::cout << "checking for " << childName << std::endl;

        //make sure child element and corresponding text exists
        TiXmlElement* childElem = elem->FirstChildElement(childName);
        if(! childElem) return true;
        const char* xmlVal = childElem->GetText();
        if(! xmlVal) return true;

        value = std::string(xmlVal);
        return false;
    }

    /**
     * @param s the string to parse for a boolean
     * @return the value of the resulting boolean
     **/
    bool stob(std::string s) {
        auto result = false;  // failure to assert is false

        std::istringstream is(s);
        // first try simple integer conversion
        is >> result;

        if (is.fail()) {
            // simple integer failed; try boolean
            is.clear();
            is >> std::boolalpha >> result;
        }
        return result;
    }

}  // namespace robotmotors

//example xml
