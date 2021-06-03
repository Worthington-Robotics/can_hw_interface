#pragma once

#include <tinyxml.h>

#include <map>
#include <string>
#include <vector>
#include <exception>

#include "genericmotor.hpp"

namespace robotmotors {
    std::shared_ptr<std::vector<robotmotors::MotorMap>> createMotorMap(TiXmlDocument* doc);
}  // namespace robotmotors