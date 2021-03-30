#pragma once

#include <tinyxml.h>

#include <map>
#include <string>
#include <vector>

#include "genericmotor.hpp"

namespace robotmotors {
    std::vector<robotmotors::MotorMap> createMotorMap(TiXmlDocument* doc);

}  // namespace robotmotors