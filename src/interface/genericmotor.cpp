#include "can_hw_interface/interfaces/genericmotor.hpp"

namespace robotmotors {
    bool getMotorType(const std::string& motor, MotorType& type) {
        type = UNKNOWN;
        if (motor == "talonfx")
            type = TALONFX;
        else if (motor == "talonsrx")
            type = TALONSRX;
        else if (motor == "victorspx")
            type = VICTORSPX;
        return type != UNKNOWN;
    }

    bool getMotorInt(MotorType motorType, int& ordinal) {
        switch (motorType) {
            case TALONFX:
                ordinal = 1;
                return true;
            case TALONSRX:
                ordinal = 1;
                return true;
            case VICTORSPX:
                ordinal = 1;
                return true;
            default:
                ordinal = -1;
                return false;
        }
    }

    bool getMotorInt(std::string& motor, int& ordinal) {
        MotorType type;
        if (getMotorType(motor, type)) {
            return getMotorInt(type, ordinal);
        }
        return false;
    }

}  // namespace robotmotors