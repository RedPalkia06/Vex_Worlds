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
    while(avgPercent < 1.0 && ti - startTime < timeout) {
        dt = Brain.Timer.time(seconds) - ti;
        ti = Brain.Timer.time(seconds);
        leftDistance = leftDistance + Constants::WHEEL_CIRC * drivetrain.LeftSide.velocity(rpm) * dt / 60.0;
        rightDistance = rightDistance + Constants::WHEEL_CIRC * drivetrain.RightSide.velocity(rpm) * dt / 60.0;
        Brain.Screen.setCursor(3, 3);
        Brain.Screen.print(leftDistance);
        Brain.Screen.print("  ");
        Brain.Screen.print(rightDistance);
        //adjust drive velocity based off of the percent difference from the average
        drivetrain.LeftSide.setVelocity(velocity + sensitivity * calculatePercentDifference(distance, leftDistance, rightDistance), percent);
        drivetrain.RightSide.setVelocity(velocity + sensitivity * calculatePercentDifference(distance, rightDistance, leftDistance), percent);
        avgPercent = (leftDistance + rightDistance) / (2.0 * distance);
    }
    drivetrain.LeftSide.stop();
    drivetrain.RightSide.stop();
    return true;
}

bool arc_to_point(brain Brain, Drivetrain drivetrain, double initialPoint[2], double finalPoint[2], double radius, double velocity, bool direction, double timeout = 10.0) {
    double turningAngle = acos(((pow(initialPoint[0] - finalPoint[0], 2) + pow(initialPoint[1] - finalPoint[1], 2)) / (-2 * pow(radius, 2))) + 1);
    Brain.Screen.setCursor(5, 5);
    Brain.Screen.print(turningAngle);
    int turningDirection = -1;
    int drivingDirection = 1;
    if (direction) {turningDirection = 1;}
    if (velocity < 0) {drivingDirection = -1;}
    //figure out the direction to turn to before beginning
    double leftDistanceTotal = drivingDirection * turningAngle * (radius + turningDirection * Constants::WHEELBASE / 2.0);
    double rightDistanceTotal = drivingDirection * turningAngle * (radius - turningDirection * Constants::WHEELBASE / 2.0);
    double angularVelocity = velocity / radius;
    double leftVelocity = angularVelocity * (radius + turningDirection * Constants::WHEELBASE / 2.0);
    double rightVelocity = angularVelocity * (radius - turningDirection * Constants::WHEELBASE / 2.0);
    double leftSensitivity = 0.01 * leftDistanceTotal;
    double rightSensitivity = 0.01 * rightDistanceTotal;
    double avgPercent = 0;
    double startTime = Brain.Timer.time(seconds);
    double ti = startTime;
    double dt = 0;
    double leftDistance = 0;
    double rightDistance = 0;
    double leftPercent = 0;
    double rightPercent = 0;
    drivetrain.LeftSide.spin(forward);
    drivetrain.RightSide.spin(forward);
    //each side of the drivetrain must drive distance cm forward
    while(avgPercent < 1.0 && ti - startTime < timeout) {
        dt = Brain.Timer.time(seconds) - ti;
        ti = Brain.Timer.time(seconds);
        leftDistance += Constants::WHEEL_CIRC * drivetrain.LeftSide.velocity(rpm) * dt / 60.0;
        rightDistance += Constants::WHEEL_CIRC * drivetrain.RightSide.velocity(rpm) * dt / 60.0;
        //calculate percents
        leftPercent = leftDistance / leftDistanceTotal;
        rightPercent = rightDistance / rightDistanceTotal;
        avgPercent = (leftPercent + rightPercent) / 2.0;
        //adjust drive velocity based off of the percent difference from the average
        drivetrain.LeftSide.setVelocity(leftVelocity + leftSensitivity * (avgPercent - leftPercent), percent);
        drivetrain.RightSide.setVelocity(rightVelocity + rightSensitivity * (avgPercent - rightPercent), percent);
    }
    drivetrain.LeftSide.stop();
    drivetrain.RightSide.stop();
    return true;
}

bool turn_to(Drivetrain drivetrain, double angle, double velocity) {
    double a = drivetrain.getRotation() / Constants::TO_RADIANS - angle;
    turnType direction;
    if(a > 180) {
        a = 360 - a;
        direction = left;
    } else if(a < -180) {
        a += 360; 
        direction = right;
    } else if(a < 0) {
        a = fabs(a);
        direction = left;
    } else {
        direction = right;
    }
    if(direction == left) {
        drivetrain.LeftSide.setVelocity(-1 * velocity, percent);
        drivetrain.RightSide.setVelocity(velocity, percent);
    } else {
        drivetrain.LeftSide.setVelocity(velocity, percent);
        drivetrain.RightSide.setVelocity(-1 * velocity, percent);
    }
    drivetrain.LeftSide.spin(forward);
    drivetrain.RightSide.spin(forward);
    while(fabs(drivetrain.getRotation() / Constants::TO_RADIANS - angle) > 0.5) {
        wait(1, msec);
    }
    drivetrain.LeftSide.stop();
    drivetrain.RightSide.stop();
    return true;
}