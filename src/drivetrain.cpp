#include "vex.h"
#include "drivetrain.h"
#include "constants.h"
using namespace vex;

double Drivetrain::max(double x, double y) {
  if (x >= y) {
    return x;
  } else {
    return y;
  }
}

double Drivetrain::curve(double x) {
  return (0.1 * x + 0.9 * pow(x, 3));
}

void Drivetrain::setMotorSpeeds(controller c) {
  //how motor speeds work: 1. curve inputs on a controller
  //2. for left side: add axis 1 for forward velocity and axis 3 to rotate robot
  //3. for right side: add axis 1 for forward velocity and subtract axis 3 for rotation
  //4. divide by the absolute value of the sum of the two axes, if it is greater than 100 divide the motor speeds by that value over 100
  //5. set the motor group to its calculated velocity
  double forwardSpeed = curve(c.Axis3.position() / 100.0);
  double turningSpeed = curve(c.Axis1.position() / 100.0);
  LeftSide.setVelocity(100.0 * ((forwardSpeed + turningSpeed) / max(fabs(forwardSpeed) + fabs(turningSpeed), 1.0)), percent);
  RightSide.setVelocity(100.0 * ((forwardSpeed - turningSpeed) / max(fabs(forwardSpeed) + fabs(turningSpeed), 1.0)), percent);
}

void Drivetrain::updatePositions(double dt) {
  Drivetrain::leftSidePosition[0] = leftSidePosition[0] + Constants::WHEEL_RADIUS * Drivetrain::LeftSide.velocity(dps) * cos(Drivetrain::getRotation()) * dt * Constants::TO_RADIANS;
  Drivetrain::rightSidePosition[0] = rightSidePosition[0] + Constants::WHEEL_RADIUS * Drivetrain::RightSide.velocity(dps) * cos(Drivetrain::getRotation()) * dt * Constants::TO_RADIANS;
  Drivetrain::position[0] = (Drivetrain::leftSidePosition[0] + Drivetrain::rightSidePosition[0]) / 2.0;
  Drivetrain::leftSidePosition[1] = leftSidePosition[1] + Constants::WHEEL_RADIUS * Drivetrain::LeftSide.velocity(dps) * sin(Drivetrain::getRotation()) * dt * Constants::TO_RADIANS;
  Drivetrain::rightSidePosition[1] = rightSidePosition[1] + Constants::WHEEL_RADIUS * Drivetrain::RightSide.velocity(dps) * sin(Drivetrain::getRotation()) * dt * Constants::TO_RADIANS;
  Drivetrain::position[1] = (Drivetrain::leftSidePosition[1] + Drivetrain::rightSidePosition[1]) / 2.0;
}
//odometry: should be inside the drivetrain class
//drivetrain should have a position for each side of the robot
//get position function should return the average of each side of the robot
//use basic code; if time, use something similar to Kalman filter