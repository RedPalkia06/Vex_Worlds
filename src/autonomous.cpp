#include "vex.h"
#include "drivetrain.h"
#include "constants.h"
#include "autonomous.h"
#include "math_utils.h"
#include "drivercontrol.h"
#include <iostream>

void six_piece_auto(Drivetrain drivetrain, pneumatics SideWing, pneumatics Wings, vex::motor Intake) {
  //robot starts at (65, 25) heading 315 degrees
  drivetrain.set_initial_position(new double[2] {25, 65});
  drivetrain.Inertial.setHeading(45, degrees);
  //release triball from corner and set up preload
  input(Intake);
  drivetrain.drive_for(35, 25, 0.75);
  Intake.stop();
  wing_down(SideWing);
  //reverse arc turn, do later
  drivetrain.turn_to(135, 15);
  intake(Intake);
  drivetrain.drive_for(10, 15, 0.5);
  Intake.stop();
  wing_up(SideWing);
  drivetrain.turn_to(315, 20);
  input(Intake);
  //get triball out of alley
  drivetrain.turn_to(90, 20);
  intake(Intake);
  drivetrain.drive_for(70, 30, 1);
  Intake.stop();
  drivetrain.drive_for(-70, -30, 1);
  drivetrain.turn_to(135, 15);
  //push 3 triballs into goal
  //reverse arc turn to (90, 23)
  drivetrain.turn_to(180, 30);
  for(int i = 0; i < 3; i++) {
    drivetrain.drive_for(-12, 100, 0.25);
    drivetrain.drive_for(12, 30, 0.4);
  }
  //get three center triballs
}