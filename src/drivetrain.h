#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H
#include "vex.h"
#include "constants.h"
#include "PID.h"
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

        PIDController turningController;

        double rightSidePosition[2] = {0, 0};
        double leftSidePosition[2] = {0, 0};

    public:

        //motor groups and sensors
        inertial Inertial = inertial(PORT7);
        motor_group LeftSide = motor_group(LeftRear, LeftMid, LeftFront);
        motor_group RightSide = motor_group(RightRear, RightMid, RightFront);
        brain brain;
        //drivetrain variables
        double position[2] = {0, 0}; //position [x, y]

        //constructor:
        Drivetrain(double kp, double ki, double kd, double t, double position_tolerance_in, double velocity_tolerance_in) : turningController(brain, kp, ki, kd, t, position_tolerance_in, velocity_tolerance_in) {
        }

        //functions: 
        void drive_for(double distance, double velocity, double timeout);
        void turn_to(int angle, double velocity);
        void turn_toPID(double angle, double velocity = 100.0); //for testing
        void arc_to_point(double finalPoint[2], double radius, double velocity, Constants::DIRECTION turn_direction, double timeout);
        void drive_to_point(double finalPoint[2], double velocity, double timeout);
        void turn_to_test(double angle, double timeout = 0.8);

        void updatePositions(double dt);
        void set_initial_position(double position[2]);
        void set_motor_speeds(controller c);
        double getRotationDegrees();
        double getRotationRadians();
};

#endif