#pragma once

#include <map>
#include <string>

namespace robotsensors {
    class GenericSensor {
    public:
        GenericSensor();

        void getType(const std::string& type);
        void configure(std::map<std::string, double> config);
        void read(double& value1, double& value2, double& value3);

        ~GenericSensor();
    };
}  // namespace sensors
