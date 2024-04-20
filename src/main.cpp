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
Drivetrain drivetrain1 = Drivetrain(0.82, 0.0, 0.0, 0.0, 5/360.0, 10.0/360.0);
pneumatics horizontal_wings = pneumatics(Brain.ThreeWirePort.A);
pneumatics vertical_wing = pneumatics(Brain.ThreeWirePort.B);
pneumatics climber = pneumatics(Brain.ThreeWirePort.H);
//Launcher launcher = Launcher()

motor Launcher = motor(PORT12, ratio36_1, true);
motor Intake = motor(PORT10, ratio6_1, false);


/*
drive_for(brain, Drivetrain, distance, velocity, timeout);
turn_to(Drivetrain, angle, velocity);
arc_to_point(brain, Drivetrain, initialPoint[2], finalPoint[2], radius, velocity, bool direction, timeout);
*/
//arc to point: true is clockwise
//current autos only use the basic autonomous functions, need to improve them for acceleration and simplicity

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

void autonomous() {
  //max_defense(drivetrain1, vertical_wing, horizontal_wings, Intake);
  five_piece(drivetrain1, vertical_wing, horizontal_wings, Intake);
}

/* Run before autonomous is initialized */
void preAutonomous() {
  drivetrain1.Inertial.calibrate();
  while(drivetrain1.Inertial.isCalibrating()) {
    wait(10, msec);
  }
  drivetrain1.Inertial.setHeading(270, degrees);
  Intake.setVelocity(100, percent);
  thread odometry = thread(odometryLoop);
  drivetrain1.LeftSide.spin(forward);
  drivetrain1.RightSide.spin(forward);
  resetLauncher(Launcher);
}

void launchLoopCallback() {
  launchLoop(Controller1, Launcher);
}
void stopIntakeCallback() {
  Intake.stop();
}
void intakeCallback() {
  Intake.spin(forward);
}
void inputCallback() {
  Intake.spin(reverse);
}

void register_controller_callbacks() {
  Controller1.ButtonL1.released(stopIntakeCallback);
  Controller1.ButtonL2.released(stopIntakeCallback);
  Controller1.ButtonL1.pressed(intakeCallback);
  Controller1.ButtonL2.pressed(inputCallback);
  
  Controller1.ButtonA.pressed([](){ horizontal_wings.open(); });
  Controller1.ButtonB.pressed([](){ horizontal_wings.close(); });
  Controller1.ButtonUp.pressed([](){ climber.open(); });
  Controller1.ButtonDown.pressed([](){ climber.close(); });
  Controller1.ButtonX.pressed([](){ vertical_wing.close(); });
  Controller1.ButtonY.pressed([](){ vertical_wing.open(); });
}

void driverControl() {
  thread launch_loop = thread(launchLoopCallback);
  drivetrain1.LeftSide.spin(forward);
  drivetrain1.RightSide.spin(forward);
  drivetrain1.LeftSide.setStopping(coast);
  drivetrain1.RightSide.setStopping(coast);

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
