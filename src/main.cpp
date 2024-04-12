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
using namespace vex;

brain Brain;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
controller Controller1 = controller(primary);
Drivetrain drivetrain1 = Drivetrain();
motor Launcher = motor(PORT12, ratio36_1, true);
motor Intake = motor(PORT10, ratio6_1, false);
pneumatics Wings = pneumatics(Brain.ThreeWirePort.A);
pneumatics SideWing = pneumatics(Brain.ThreeWirePort.B);
pneumatics Climber = pneumatics(Brain.ThreeWirePort.H);
/* Autonomous Code here 
*  1 = No Auto
*  2 = Offense
*  3 = Defense
*  4 = Skills
*/

//arc to point: true is right
void autonomous() {
  double position[2] = {200, 200};
  arc_to_point(Brain, drivetrain1, drivetrain1.position, position, 290, 10, true, 10);
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
  }
}

/* Run before autonomous is initialized */
void preAutonomous() {
  drivetrain1.Inertial.setRotation(90, degrees);
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
  stopIntake(Intake);
}
void openWingsCallback() {
  openWings(Wings);
}
void closeWingsCallback() {
  closeWings(Wings);
}
void launchLoopCallback() {
  launchLoop(Controller1, Launcher);
}
void climbCallback() {
  climb(Climber);
}
void climbDownCallback() {
  climbDown(Climber);
}
void wingDownCallback() {
  wingDown(SideWing);
}
void wingUpCallback() {
  wingUp(SideWing);
}

void driverControl() {
  Controller1.ButtonL1.released(stopIntakeCallback);
  Controller1.ButtonL2.released(stopIntakeCallback);
  Controller1.ButtonL1.pressed(intakeCallback);
  Controller1.ButtonL2.pressed(inputCallback);
  Controller1.ButtonA.pressed(openWingsCallback);
  Controller1.ButtonB.pressed(closeWingsCallback);
  Controller1.ButtonUp.pressed(climbCallback);
  Controller1.ButtonDown.pressed(climbDownCallback);
  Controller1.ButtonX.pressed(wingDownCallback);
  Controller1.ButtonY.pressed(wingUpCallback);
  thread LaunchLoop = thread(launchLoopCallback);
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    drivetrain1.setMotorSpeeds(Controller1);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
