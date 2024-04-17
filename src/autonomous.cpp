#include "autonomous.h"

/*
void six_piece_auto(Drivetrain drivetrain, PneumaticWing side_wing, PneumaticWing wings, vex::motor Intake) {
  //robot starts at (65, 25) heading 315 degrees
  drivetrain.set_initial_position(new double[2] {25, 65});
  drivetrain.Inertial.setHeading(45, degrees);
  //release triball from corner and set up preload
  input(Intake);
  drivetrain.drive_for(35, 25, 0.75);
  Intake.stop();
  side_wing.wings_out();
  //reverse arc turn, do later
  drivetrain.turn_to(135, 15);
  intake(Intake);
  drivetrain.drive_for(10, 15, 0.5);
  Intake.stop();
  side_wing.wings_in();
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
  for(int i = 0; i < 2; i++) {
    drivetrain.drive_for(-12, 100, 0.25);
    drivetrain.drive_for(12, 30, 0.4);
  }
  //get three center triballs
  //drive to back center, intake, spit
  //drive to left, intake, spit
  //turn around, open wings, push into goal
}
*/
/*
  Drive forward
  Spit preoload
  Wing down
  Spin towards alley
  Grab triball
  spit towards goal
  back into goal
  shove x2
  drive to center triball
  intake
  turn to goal
  spit
  turn to left triball
  drive forwards
  turn towards goal
  spit
  turn backwards
  put out wings
  drive back into goal
  touch bar:
    drive to inside corner
    put down side wing
    turn
*/
void six_piece(Drivetrain drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake) {
  drivetrain.set_initial_position(new double[2] {25, 65});
  drivetrain.Inertial.setHeading(45, degrees);
  vertical_wing.open();
  intake.spin(reverse);
  drivetrain.drive_for(75, 50, 0.5);
  drivetrain.turn_toPID(0, 100);
  drivetrain.drive_for(40, 50, 0.5);
  //replace with arc turn later
  intake.stop();
  vertical_wing.close();
  drivetrain.drive_for(-20, -25, 0.5);
  drivetrain.turn_toPID(135, 100);
  drivetrain.drive_for(80, 50, 0.5);
  drivetrain.turn_toPID(90, 25);
  intake.spin(forward);
  drivetrain.drive_for(100, 60, 0.5);
  drivetrain.turn_toPID(270, 100);
  intake.spin(reverse);
  wait(0.25, seconds);
  drivetrain.turn_toPID(90, 100);
  intake.stop();
  drivetrain.drive_for(-100, -40, 0.5);
  drivetrain.turn_toPID(135, 100);
  drivetrain.drive_for(-50, 50, 0.5);
  drivetrain.arc_to_point(new double[2] {65, 35}, 45, -50, Constants::COUNTERCLOCKWISE, 0.5);
  drivetrain.drive_for(10, 100, 0.1);
  drivetrain.drive_for(-20, -100, 0.2);
  drivetrain.drive_for(10, 30, 0.5);

  //drive to center triball
  
}

/*
  Defense (max points):
  Start towards goal on angle bar
  drive forward
  spit
  reverse arc turn into goal
  arc turn to center of field
  face towards goal
  put out wings
  drive backwards at 100% and knock triballs out of side
  put in wings
  turn to corner of field
  drive forward
  turn to alley/diagonal
  put out side wing
  kick out triball
  turn around
  back into alley
  touch bar
*/

/*
  Defense (safe)
  Start towards goal on angle bar
  drive forward
  spit
  reverse arc turn into goal
  put out side wing
  kick out triball
  turn around
  back into alley
  touch bar
*/

/*
  Skills auto
  Lock into side bar with wings
  (launch preload?)
  Fire until 23 seconds
  IMPORTANT: RESET INERTIAL SENSOR (calibrate) AND INITIAL POSITION
  Wings in
  drive forward a bit
  Wings out
  Reverse arc to middle-right of center bar
  Forward arc to in front of middle-left of center bar
  Drive into bar
  Wings in
  Drive forward
  Arc turn into alley (180 degree turn, 0 radius)
  Drive forward while intaking
  Stop intake (still driving forward, use thread)
  Spit out triball
  Reverse arc turn into goal
  Shove x1
  Forward arc while outtaking to middle-left of center bar
  Turn towards goal (slight angle)
  Wings out
  Drive into goal backwards
  Wings in
  Turn to 90 degrees
  Drive reverse to center bar
  Turn to ~60 degrees
  Drive forward a bit
  Turn at angle towards goal
  Wings out
  Drive into goal
  Shove
  Wings in
  Drive out of goal
  (if time:)
  Reverse 180 degree arc into right side
  Short 90 degree arc into side of goal
  Shove x2
  Drive forward
  (if a lot more time:)
  Climber up
  Turn to alley
  Drive to bar
  Close climber
*/