#include "vex.h"
#include "drivetrain.h"
#include "constants.cpp"
using namespace vex;

double Drivetrain::max(double x, double y) {
  if (x >= y) {
    return x;
  } else {
    return y;
  }
}

double Drivetrain::curve(double x) {
  return (0.2 * x + 0.8 * pow(x, 3));
}

void Drivetrain::setMotorSpeeds(vex::controller c) {
  //how motor speeds work: 1. curve inputs on a controller
  //2. for left side: add axis 1 for forward velocity and axis 3 to rotate robot
  //3. for right side: add axis 1 for forward velocity and subtract axis 3 for rotation
  //4. divide by the absolute value of the sum of the two axes, if it is greater than 100 divide the motor speeds by that value over 100
  //5. set the motor group to its calculated velocity
  LeftSide.setVelocity(100.0 * ((curve(c.Axis1.position()) + curve(c.Axis3.position())) / max(fabs(curve(c.Axis1.position()) + curve(c.Axis3.position())), 100.0)), percent);
  RightSide.setVelocity(100.0 * ((curve(c.Axis1.position()) - curve(c.Axis3.position())) / max(fabs(curve(c.Axis1.position()) + curve(c.Axis3.position())), 100.0)), percent);
}

void Drivetrain::updatePositions(double dt) {
  Drivetrain::leftSidePosition[1] = leftSidePosition[1] + Constants::WHEEL_RADIUS * Drivetrain::LeftSide.velocity(dps) * Constants::TO_RADIANS * dt;
}
//odometry: should be inside the drivetrain class
//drivetrain should have a position for each side of the robot
//get position function should return the average of each side of the robot
//use basic code; if time, use something similar to Kalman filter