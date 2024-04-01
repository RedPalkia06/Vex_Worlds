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
using namespace vex;

brain Brain;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
controller Controller1 = controller(primary);
Drivetrain drivetrain = Drivetrain();

/* Run before autonomous is initialized */
void preAutonomous() {
  //reset catapult
  //make sure wings are closed
  //reset climber, if necessary
}

/* Autonomous Code here 
*  1 = No Auto
*  2 = Offense
*  3 = Defense
*  4 = Skills
*/
void autonomous() {
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
void driverControl() {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.


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
