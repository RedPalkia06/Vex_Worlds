#include "vex.h"
#include "drivetrain.h"
#include "constants.h"
#include "autonomous.h"

//separate programs for each auto unless time to make buttons for different autos
//drive for function, auto corrects

//calculatePercentDifference: determines the difference in percentage from the average percentage of distance travelled
double calculatePercentDifference(double totalDistance, double calcDistance, double otherDistance) {
    double calcPercent = calcDistance / totalDistance;
    double otherPercent = otherDistance / totalDistance;
    double avgPercent = (calcPercent + otherPercent) / 2.0;
    return (avgPercent - calcPercent) * 100.0;
}

bool drive_for(brain Brain, Drivetrain drivetrain, double distance, double velocity, double timeout = 10.0) {
    double sensitivity = 0.01 * distance;
    drivetrain.LeftSide.spin(forward);
    drivetrain.RightSide.spin(forward);
    double avgPercent = 0;
    double leftDistance = 0;
    double rightDistance = 0;
    double startTime = Brain.Timer.time(seconds);
    double ti = Brain.Timer.time(seconds);
    double dt = 0;
    //each side of the drivetrain must drive distance cm forward
    while(avgPercent < 1.0 && ti - startTime < 10.0) {
        dt = Brain.Timer.time(seconds) - ti;
        ti = Brain.Timer.time(seconds);
        leftDistance = leftDistance + Constants::WHEEL_CIRC * drivetrain.LeftSide.velocity(rpm) * dt / 60.0;
        rightDistance = rightDistance + Constants::WHEEL_CIRC * drivetrain.RightSide.velocity(rpm) * dt / 60.0;
        //adjust drive velocity based off of the percent difference from the average
        drivetrain.LeftSide.setVelocity(velocity + sensitivity * calculatePercentDifference(distance, leftDistance, rightDistance), percent);
        drivetrain.RightSide.setVelocity(velocity + sensitivity * calculatePercentDifference(distance, rightDistance, leftDistance), percent);
        avgPercent = (leftDistance + rightDistance) / (2.0 * distance);
    }
    drivetrain.LeftSide.stop();
    drivetrain.RightSide.stop();
    return true;
}

//drive to point functions:
//take in current position {x, y} of robot
//target position {x, y}
//turn to point function?
//drive corrected distance to the point

//arced turn:
//determine distance to travel by each side of the robot using the wheelbase
//distance: r +- wheelbase times theta, theta in radians 

//arc to point:
//take in robot's current position, target position, and circle radius

//