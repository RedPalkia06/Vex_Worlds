#include "vex.h"

class Constants {
    public:
        static const float CONTROLLER_SENSITIVITY = 0.8;
        static const float PI = 3.1416;
        static const float WHEEL_RADIUS = 9.525 / 2; //in cm
        static const float WHEEL_CIRC = 2 * WHEEL_RADIUS * PI;
        static const float TO_RADIANS = PI / 180.0;
};