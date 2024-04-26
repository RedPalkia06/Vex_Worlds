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

Constants::AUTONOMOUS autonomous_index = Constants::AUTONOMOUS::NONE;

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
  if (autonomous_index == Constants::AUTONOMOUS::NONE) {}
  else if (autonomous_index == Constants::AUTONOMOUS::DISRUPT_DEFENSE) {
    max_defense(drivetrain1, vertical_wing, horizontal_wings, Intake);
  } else if (autonomous_index == Constants::AUTONOMOUS::OFFENSE) {
    five_piece(drivetrain1, vertical_wing, horizontal_wings, Intake);
  }else if (autonomous_index == Constants::AUTONOMOUS::SAFE_DEFENSE) {
    safe_defense(drivetrain1, vertical_wing, horizontal_wings, Intake);
  } else if (autonomous_index == Constants::AUTONOMOUS::SKILLS) {
    auto_skills(drivetrain1, vertical_wing, horizontal_wings, Intake, Launcher, climber);
  }
  
   
}

/* Run before autonomous is initialized */
void preAutonomous() {
  drivetrain1.Inertial.calibrate();
  while(drivetrain1.Inertial.isCalibrating()) {
    wait(10, msec);
  }

  int column_width = 480.0 / 5.0;

  


  while(true) {
    Brain.Screen.setFillColor(color::blue);
    Brain.Screen.drawRectangle(0, 0, column_width, 272);
    
    Brain.Screen.setFillColor(color::red);
    Brain.Screen.drawRectangle(column_width, 0, column_width, 272);
    Brain.Screen.setFillColor(color::orange);
    Brain.Screen.drawRectangle(2 * column_width, 0, column_width, 272);
    Brain.Screen.setFillColor(color::purple);
    Brain.Screen.drawRectangle(3 * column_width, 0, column_width, 272);
    Brain.Screen.setFillColor(color::yellow);
    Brain.Screen.drawRectangle(4 * column_width, 0, column_width, 272);
    Brain.Screen.setFillColor(color::black);
    
    while (!Brain.Screen.pressing()) {}

    double press_x = Brain.Screen.xPosition();
    double press_y = Brain.Screen.yPosition();

    std::cout << "X: " << press_x << std::endl;
    std::cout << "Y: " << press_y << std::endl;
    std::cout << "Width: " << column_width << std::endl;

  //  480*272


    if (0 < press_x && press_x < column_width) {
      autonomous_index = Constants::AUTONOMOUS::NONE;
    } else if (column_width < press_x && press_x < 2 * column_width) {
      autonomous_index = Constants::AUTONOMOUS::DISRUPT_DEFENSE;
    } else if (2 * column_width < press_x && press_x < 3 * column_width) {
      autonomous_index = Constants::AUTONOMOUS::OFFENSE;
    } else if (3 * column_width < press_x && press_x < 4 * column_width) {
      autonomous_index = Constants::AUTONOMOUS::SAFE_DEFENSE;
    } else if (4 * column_width < press_x && press_x < 5 * column_width) {
      autonomous_index = Constants::AUTONOMOUS::SKILLS;
    } else {
      std::cout << "This should never happen" << std::endl;
    }

    switch (autonomous_index)
    {
    case Constants::AUTONOMOUS::NONE:
      Brain.Screen.print("None");
      break;
    case Constants::AUTONOMOUS::DISRUPT_DEFENSE :
      Brain.Screen.print("Distrupt Defense");
      break;
    case Constants::AUTONOMOUS::OFFENSE :
      Brain.Screen.print("Offense");
      break;
    case Constants::AUTONOMOUS::SAFE_DEFENSE :
      Brain.Screen.print("Safe Defense");
      break;
    case Constants::AUTONOMOUS::SKILLS :
      Brain.Screen.print("Skills");
      break;
    }
    Brain.Screen.newLine();
    Brain.Screen.print("Press A to accept, B to cancel");
    Brain.Screen.newLine();

    while (!Controller1.ButtonA.pressing() && !Controller1.ButtonB.pressing()) {}
    if (Controller1.ButtonA.pressing()) {
      Brain.Screen.print("Finalized autonomous");
      wait(1000, msec);
      break;
    } else if (Controller1.ButtonB.pressing()) {
      Brain.Screen.print("Canceled Selection");
      wait(1000, msec);
    }
    Brain.Screen.clearScreen();
  }


  drivetrain1.Inertial.setHeading(270, degrees);
  Intake.setVelocity(100, percent);
  thread odometry = thread(odometryLoop);
  drivetrain1.LeftSide.spin(forward);
  drivetrain1.RightSide.spin(forward);
}

void endLaunchingCallback() {
  endLaunching(Launcher);
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

  resetLauncher(Launcher);
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
