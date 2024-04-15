#ifndef CONSTANTS_H
#define CONSTANTS_H

class Constants {
    public:
        Constants() {}
        static constexpr const float CONTROLLER_SENSITIVITY = 0.8;
        static constexpr const float PI = 3.1416;
        static constexpr const float WHEEL_RADIUS = (609.0 / 624.0) * (520.0 / 620.0) * 9.525 / 2.0;
        static constexpr const float WHEEL_CIRC = WHEEL_RADIUS * 2.0 * PI;
        static constexpr const float TO_RADIANS = PI / 180.0;
        static constexpr const float WHEELBASE = 30.0;
        const enum DIRECTION { CLOCKWISE, COUNTERCLOCKWISE };
};

#endif