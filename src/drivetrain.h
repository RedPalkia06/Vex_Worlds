#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H
#include "vex.h"
#include "constants.cpp"
using namespace vex;
class Drivetrain {
    private:
      //define motors for the drivetrain
        motor LeftRear = motor(PORT4, ratio6_1, false);
        motor LeftMid = motor(PORT5, ratio6_1, false);
        motor LeftFront = motor(PORT6, ratio6_1, false);
        motor RightRear = motor(PORT1, ratio6_1, true);
        motor RightMid = motor(PORT2, ratio6_1, true);
        motor RightFront = motor(PORT3, ratio6_1, true);

        double rightSidePosition[2] = {0, 0};
        double leftSidePosition[2] = {0, 0};

        double curve(double x);
        double max(double x, double y);
        double getRotation() {
            return (-Inertial.angle(degrees)) * Constants::TO_RADIANS;
        }

    public:

        //motor groups and sensors
        inertial Inertial = inertial(PORT7);
        motor_group LeftSide = motor_group(LeftRear, LeftMid, LeftFront);
        motor_group RightSide = motor_group(RightRear, RightMid, RightFront);

        //drivetrain variables
        double position[2] = {0, 0}; //position [x, y]

        //constructor:
        Drivetrain() {}

        //functions: 
        void updatePositions(double dt);
        void setMotorSpeeds(controller c);
};

#endif