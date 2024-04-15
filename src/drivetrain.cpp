#include "vex.h"
#include "drivetrain.h"
#include <iostream>
#include "constants.h"
#include "math_utils.h"
using namespace vex;


double Drivetrain::getRotationDegrees() {
    return 360 - Inertial.heading(degrees);
}

double Drivetrain::getRotationRadians() {
    return getRotationDegrees() * Constants::TO_RADIANS;
}

void Drivetrain::set_initial_position(double pos[2]) {
  position[0] = pos[0];
  position[1] = pos[1];
}

void Drivetrain::set_motor_speeds(controller c) {
  //how motor speeds work: 1. curve inputs on a controller
  //2. for left side: add axis 1 for forward velocity and axis 3 to rotate robot
  //3. for right side: add axis 1 for forward velocity and subtract axis 3 for rotation
  //4. divide by the absolute value of the sum of the two axes, if it is greater than 100 divide the motor speeds by that value over 100
  //5. set the motor group to its calculated velocity
  double forwardSpeed = Math_Utils::curve(c.Axis3.position() / 100.0);
  double turningSpeed = Math_Utils::curve(c.Axis1.position() / 100.0);
  LeftSide.setVelocity(100.0 * ((forwardSpeed + turningSpeed) / Math_Utils::max(fabs(forwardSpeed) + fabs(turningSpeed), 1.0)), percent);
  RightSide.setVelocity(100.0 * ((forwardSpeed - turningSpeed) / Math_Utils::max(fabs(forwardSpeed) + fabs(turningSpeed), 1.0)), percent);
}

void Drivetrain::updatePositions(double dt) {
  leftSidePosition[0] = leftSidePosition[0] + Constants::WHEEL_CIRC * RightSide.velocity(rpm) * dt * cos(getRotationDegrees())/ 60.0;
  rightSidePosition[0] = rightSidePosition[0] + Constants::WHEEL_CIRC * LeftSide.velocity(rpm) * dt * cos(getRotationDegrees())/ 60.0;
  position[0] = (leftSidePosition[0] + rightSidePosition[0]) / 2.0;
  leftSidePosition[1] = leftSidePosition[1] + Constants::WHEEL_CIRC * RightSide.velocity(rpm) * dt * sin(getRotationDegrees())/ 60.0;
  rightSidePosition[1] = rightSidePosition[1] + Constants::WHEEL_CIRC * LeftSide.velocity(rpm) * dt * sin(getRotationDegrees())/ 60.0;
  position[1] = (leftSidePosition[1] + rightSidePosition[1]) / 2.0;
}
//odometry: should be inside the drivetrain class
//drivetrain should have a position for each side of the robot
//get position function should return the average of each side of the robot
//use basic code; if time, use something similar to Kalman filter

void Drivetrain::drive_for(double distance, double velocity, double timeout = 10.0) {
    double sensitivity = 0.01 * distance;
    LeftSide.spin(forward);
    RightSide.spin(forward);
    double avgPercent = 0;
    double leftDistance = 0;
    double rightDistance = 0;
    double startTime = brain.Timer.time(seconds);
    double ti = brain.Timer.time(seconds);
    double dt = 0;
    //each side of the drivetrain must drive distance cm forward
    while(avgPercent < 1.0 && ti - startTime < timeout) {
        dt = brain.Timer.time(seconds) - ti;
        ti = brain.Timer.time(seconds);
        leftDistance = leftDistance + Constants::WHEEL_CIRC * LeftSide.velocity(rpm) * dt / 60.0;
        rightDistance = rightDistance + Constants::WHEEL_CIRC * RightSide.velocity(rpm) * dt / 60.0;
        brain.Screen.setCursor(3, 3);
        brain.Screen.print(leftDistance);
        brain.Screen.print("  ");
        brain.Screen.print(rightDistance);
        //adjust drive velocity based off of the percent difference from the average
        LeftSide.setVelocity(velocity + sensitivity * Math_Utils::calculatePercentDifference(distance, leftDistance, rightDistance), percent);
        RightSide.setVelocity(velocity + sensitivity * Math_Utils::calculatePercentDifference(distance, rightDistance, leftDistance), percent);
        avgPercent = (leftDistance + rightDistance) / (2.0 * distance);
    }
    LeftSide.stop();
    RightSide.stop();
}


/**
 * Arcs the robot from its current point to a target point, given a radius
 * @param velocity Negative for backwards turns.
 * @param turn_direction The direction of the arc
 * @param timeout How long the program will run before shutting off
 **/
void Drivetrain::arc_to_point(double finalPoint[2], double radius, double velocity, Constants::DIRECTION turn_direction, double timeout = 10.0) {
    double initialPoint[2] = {position[0], position[1]};
    double distanceBetweenPoints = sqrt(pow(initialPoint[0] - finalPoint[0], 2) + pow(initialPoint[1] - finalPoint[1], 2));
    if(radius < distanceBetweenPoints / 2.0) {
        radius = distanceBetweenPoints / 2.0;
        brain.Screen.setCursor(5, 1);
        brain.Screen.print("Arc turn radius too small, using minimum radius of 0.5 * distance");
    }
    int direction = -1;
    if(turn_direction == Constants::CLOCKWISE) {direction = 1;}
    double centerX = Math_Utils::circleIntersectionX(initialPoint, finalPoint, radius, distanceBetweenPoints, direction);
    double centerY = Math_Utils::circleIntersectionY(initialPoint, finalPoint, radius, distanceBetweenPoints, direction);
    double turnToAngle = -1 * atan2((centerX - initialPoint[0]),(centerY - initialPoint[1])) / Constants::TO_RADIANS; 
    if(direction == 1) {
        turnToAngle += 180;
    }
    std::cout << centerX << " " << centerY << " " << turnToAngle << "\n";
    turn_to(turnToAngle, velocity);
    double turningAngle = acos((pow(distanceBetweenPoints, 2) / (-2 * pow(radius, 2))) + 1);
    brain.Screen.setCursor(5, 5);
    brain.Screen.print(turningAngle);


    //figure out the direction to turn to before beginning
    double leftDistanceTotal = turningAngle * (radius + turn_direction * Constants::WHEELBASE / 2.0);
    double rightDistanceTotal = turningAngle * (radius - turn_direction * Constants::WHEELBASE / 2.0);
    double angularVelocity = velocity / radius;
    double leftVelocity = angularVelocity * (radius + turn_direction * Constants::WHEELBASE / 2.0);
    double rightVelocity = angularVelocity * (radius - turn_direction * Constants::WHEELBASE / 2.0);

    //normalize the velocities
    if(leftVelocity > 100 && leftVelocity > rightVelocity) {
        leftVelocity = 100;
        rightVelocity = rightVelocity * 100 / leftVelocity;
    } else if(rightVelocity > 100) {
        rightVelocity = 100;
        leftVelocity = leftVelocity * 100 / rightVelocity;
    }

    //correct for the robot driving backwards
    if(velocity < 0) {
        leftDistanceTotal *= -1;
        rightDistanceTotal *= -1;
        angularVelocity *= -1;
        double tempVelocity = leftVelocity;
        leftVelocity = rightVelocity;
        rightVelocity = tempVelocity;
        turnToAngle += 180;
    }

    double leftSensitivity = 0.01 * fabs(leftDistanceTotal);
    double rightSensitivity = 0.01 * fabs(rightDistanceTotal);
    double avgPercent = 0;
    double startTime = brain.Timer.time(seconds);
    double ti = startTime;
    double dt = 0;
    double leftDistance = 0;
    double rightDistance = 0;
    double leftPercent = 0;
    double rightPercent = 0;
    LeftSide.spin(forward);
    RightSide.spin(forward);
    //each side of the drivetrain must drive distance cm forward
    while(avgPercent < 1.0 && ti - startTime < timeout) {
        dt = brain.Timer.time(seconds) - ti;
        ti = brain.Timer.time(seconds);
        leftDistance += Constants::WHEEL_CIRC * LeftSide.velocity(rpm) * dt / 60.0;
        rightDistance += Constants::WHEEL_CIRC * RightSide.velocity(rpm) * dt / 60.0;
        //calculate percents
        leftPercent = leftDistance / leftDistanceTotal;
        rightPercent = rightDistance / rightDistanceTotal;
        avgPercent = (leftPercent + rightPercent) / 2.0;
        //adjust drive velocity based off of the percent difference from the average
        LeftSide.setVelocity(leftVelocity + leftSensitivity * (avgPercent - leftPercent), percent);
        RightSide.setVelocity(rightVelocity + rightSensitivity * (avgPercent - rightPercent), percent);
    }
    LeftSide.stop();
    RightSide.stop();
}

void Drivetrain::turn_to(int angle, double velocity) {
    double angular_difference = Math_Utils::calculate_optimal_turn(getRotationDegrees(), angle);
    double target_angle = getRotationDegrees() + angular_difference;
    
    LeftSide.spin(forward);
    RightSide.spin(forward);
    double delta_heading = Math_Utils::calculate_optimal_turn(getRotationDegrees(), target_angle);
    while(fabs(delta_heading) > 0.5) {
        delta_heading = Math_Utils::calculate_optimal_turn(getRotationDegrees(), target_angle);

        std::cout << "CH: " << getRotationDegrees() << std::endl;
        std::cout << "DH: " << angular_difference << std::endl;
        double power = delta_heading * -0.02;
        LeftSide.setVelocity(power * velocity, percent);
        RightSide.setVelocity(-power * velocity, percent);
        wait(10, msec);
    }
    LeftSide.stop();
    RightSide.stop();
}