#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "vex.h"
using namespace vex;

class Constants {
public:
    Constants() {}

    static constexpr float CONTROLLER_SENSITIVITY = 0.8;
    static constexpr float PI = 3.1416;
    static constexpr float WHEEL_RADIUS = (609.0 / 624.0) * (520.0 / 620.0) * 9.525 / 2.0;
    static constexpr float WHEEL_CIRC = WHEEL_RADIUS * 2.0 * PI;
    static constexpr float TO_RADIANS = PI / 180.0;
    static constexpr float WHEELBASE = 30.0;
    static constexpr float TURNING_CONSTANT = 0.8;
    enum DIRECTION { CLOCKWISE, COUNTERCLOCKWISE };
    enum AUTONOMOUS { NONE, DISRUPT_DEFENSE,  SAFE_DEFENSE,  OFFENSE,  SKILLS};

};

#endif
