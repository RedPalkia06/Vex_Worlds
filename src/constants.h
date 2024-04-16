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
    enum DIRECTION { CLOCKWISE, COUNTERCLOCKWISE };
    
    // Define triport as a static member of the Constants class
    static triport triport_;

    // Define the triport ports using the triport object
    static const triport::port HORIZONTAL_WINGS_SOLENOID_PORT;
    static const triport::port VERTICAL_WING_SOLENOID_PORT;
    static const triport::port CLIMBER_SOLENOID_PORT;

};

// Define the triport object outside the class and initialize the ports
triport Constants::triport_ = triport(Brain.ThreeWirePort.A);

// Define the triport port outside the class
const triport::port Constants::HORIZONTAL_WINGS_SOLENOID_PORT = triport_.A;
const triport::port Constants::VERTICAL_WING_SOLENOID_PORT = triport_.B;
const triport::port Constants::CLIMBER_SOLENOID_PORT = triport_.C;

#endif
