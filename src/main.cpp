/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       aislo                                                     */
/*    Created:      3/7/2024, 6:07:47 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "drivetrain.h"
#include "drivercontrol.h"
#include "autonomous.h"
#include "pneumatic_wings.h"
#include "climber.h"
using namespace vex;

brain Brain;
competition Competition;
controller Controller1 = controller(primary);

// Subsystems
Drivetrain drivetrain1 = Drivetrain();
PneumaticWing horizontal_wings = PneumaticWing(Constants::HORIZONTAL_WINGS_PORT);
PneumaticWing veritcal_wing = PneumaticWing(Brain.ThreeWirePort.B);
Climber climber = Climber(Brain.ThreeWirePort.H);
Launcher launcher = Launcher()

motor Launcher = motor(PORT12, ratio36_1, true);
motor Intake = motor(PORT10, ratio6_1, false);





/*
drive_for(brain, Drivetrain, distance, velocity, timeout);
turn_to(Drivetrain, angle, velocity);
arc_to_point(brain, Drivetrain, initialPoint[2], finalPoint[2], radius, velocity, bool direction, timeout);
*/
//arc to point: true is clockwise
//current autos only use the basic autonomous functions, need to improve them for acceleration and simplicity

void autonomous() {
  drivetrain1.LeftSide.setStopping(brake);
  drivetrain1.RightSide.setStopping(brake);
  drivetrain1.arc_to_point(new double[2] {100, 200}, 0, 10, Constants::CLOCKWISE, 10);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void odometryLoop() {
  double dt = 0;
  double ti = Brain.Timer.time(seconds);
  while(1) {
    dt = Brain.Timer.time(seconds) - ti;
    ti = Brain.Timer.time(seconds);
    drivetrain1.updatePositions(dt);
    wait(0.01, seconds);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(drivetrain1.position[0]);
    Brain.Screen.newLine();
    Brain.Screen.print(drivetrain1.position[1]);
    Brain.Screen.newLine();
    Brain.Screen.print(drivetrain1.getRotationDegrees());
  }
}

/* Run before autonomous is initialized */
void preAutonomous() {
  drivetrain1.Inertial.calibrate();
  while(drivetrain1.Inertial.isCalibrating()) {
    wait(10, msec);
  }
  drivetrain1.Inertial.setHeading(270, degrees);
  Intake.setVelocity(100, percent);
  drivetrain1.LeftSide.spin(forward);
  drivetrain1.RightSide.spin(forward);
  thread odometry = thread(odometryLoop);
  //make sure wings are closed
  //reset climber, if necessary
}

//callbacks
void intakeCallback() {
  intake(Intake);
}
void inputCallback() {
  input(Intake);
}
void stopIntakeCallback() {
  stop_intake(Intake);
}
void launchLoopCallback() {
  launchLoop(Controller1, Launcher);
}
void climbCallback() {
  climb(Climber);
}
void climbDownCallback() {
  climb_down(Climber);
}


void register_controller_callbacks() {
  Controller1.ButtonL1.released(stopIntakeCallback);
  Controller1.ButtonL2.released(stopIntakeCallback);
  Controller1.ButtonL1.pressed(intakeCallback);
  Controller1.ButtonL2.pressed(inputCallback);
  Controller1.ButtonA.pressed([](){ horizontal_wings.out(); });
  Controller1.ButtonB.pressed([](){ horizontal_wings.in(); });
  Controller1.ButtonUp.pressed(climbCallback);
  Controller1.ButtonDown.pressed(climbDownCallback);
  Controller1.ButtonX.pressed([](){ veritcal_wing.down(); });
  Controller1.ButtonY.pressed([](){ veritcal_wing.up(); });
}

void driverControl() {
  thread LaunchLoop = thread(launchLoopCallback);

  while (1) {
    drivetrain1.set_motor_speeds(Controller1);
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//

int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(driverControl);

  // Run the pre-autonomous function.
  preAutonomous();
  register_controller_callbacks();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
