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
void intake_delay() {
  motor intake = motor(PORT10, ratio6_1, false);
  intake.setVelocity(100, percent);
  wait(0.5, seconds);
  intake.spin(reverse);
}

void wing_delay() {
  brain Brain;
  pneumatics wing = pneumatics(Brain.ThreeWirePort.B);
  wait(0.4, seconds);
  wing.open();
}

void five_piece(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake) {
  brain brain;
  double t_i = brain.Timer.time(seconds);
  //drivetrain.set_initial_position(new double[2] {25, 90});
  //drivetrain.Inertial.setHeading(270, degrees);
  drivetrain.set_initial_position(new double[2] {25, 62});
  drivetrain.Inertial.setHeading(45, degrees);
  vertical_wing.open();
  drivetrain.drive_for(15, 60, 0.6);
  drivetrain.turn_to_test(22);
  intake.spin(reverse);
  drivetrain.drive_for(55, 100, 0.7);
  drivetrain.drive_for(-20, -60, 0.5);
  vertical_wing.close();
  intake.spin(forward);
  drivetrain.turn_to_test(270);
  wait(0.3, seconds);
  drivetrain.turn_to_test(340);
  intake.spin(reverse);
  drivetrain.drive_for(50, 100, 0.7);
  intake.stop();
  drivetrain.drive_for(-40, -50, 0.5);
  intake.spin(forward);
  drivetrain.drive_to_point(new double[2] {90, 100}, 60, 0.7);
  drivetrain.drive_to_point(new double[2] {100, 150}, 60, 0.6);
  drivetrain.turn_to_test(275, 1.2);
  intake.setVelocity(70, percent);
  intake.spin(reverse);
  wait(0.6, seconds);
  intake.setVelocity(100, percent);
  intake.spin(forward);
  drivetrain.drive_to_point(new double[2] {147, 150}, 80, 0.6);
  drivetrain.turn_to_test(260, 1);
  intake.setVelocity(70, percent);
  intake.spin(reverse);
  wait(0.5, seconds);
  drivetrain.turn_toPID(80);
  intake.stop();
  intake.setVelocity(100, percent);
  horizontal_wing.open();
  drivetrain.drive_for(-120, -100, 0.8);
  horizontal_wing.close();
  drivetrain.drive_for(20, 20, 0.5);
  brain.Screen.setCursor(10, 10);
  brain.Screen.print(brain.Timer.time(seconds) - t_i);
  drivetrain.LeftSide.setStopping(coast);
  drivetrain.RightSide.setStopping(coast);
  return;

  //drive to center triball
  
}
void safe_auto(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake) {
  brain brain;
  double t_i = brain.Timer.time(seconds);
  drivetrain.set_initial_position(new double[2] {35, 75});
  drivetrain.Inertial.setHeading(215, degrees);
  drivetrain.drive_for(-50, -10, 3);
  drivetrain.turn_toPID(180);
  drivetrain.drive_for(-70, -100, 0.6);
  drivetrain.drive_for(50, 40, 0.8);
  drivetrain.drive_for(-60, -100, 1);
  drivetrain.drive_for(30, 20, 3);
  brain.Screen.setCursor(10, 10);
  brain.Screen.print(brain.Timer.time(seconds) - t_i);
  drivetrain.LeftSide.setStopping(coast);
  drivetrain.RightSide.setStopping(coast);
  return;

  //drive to center triball
  
}

void max_defense(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake) {
  brain brain;
  double t_i = brain.Timer.time(seconds);
  drivetrain.set_initial_position(new double[2] {110, 35});
  drivetrain.Inertial.setHeading(90, degrees);
  intake.spin(forward);
  drivetrain.drive_for(-5, -50, 0.2);
  wait(0.3, seconds);
  drivetrain.drive_to_point(new double[2] {108, 105}, -30, 1);
  drivetrain.turn_to_test(205);
  horizontal_wing.open();
  drivetrain.drive_for(-20, -80, 1);
  drivetrain.turn_to_test(165, 1.3);
  intake.setVelocity(60, percent);
  intake.spin(reverse);
  wait(0.2, seconds);
  drivetrain.drive_for(-60, -100, 0.7);
  horizontal_wing.close();
  drivetrain.turn_toPID(0, 1.3);
  drivetrain.drive_for(-100, -50, 1.1);
  drivetrain.drive_for(20, 30, 0.35);
  //
  intake.stop();
  drivetrain.drive_to_point(new double[2] {110, 85}, 60, 1); //70 35
  drivetrain.drive_to_point(new double[2] {70, 90}, -50, 1.2);
  drivetrain.turn_to_test(305);
  vertical_wing.open();
  drivetrain.drive_to_point(new double[2] {85, 30}, 40, 0.8);
  drivetrain.turn_to_test(20);
  drivetrain.drive_for(30, 50, 0.7);
  drivetrain.drive_for(-30, -50, 0.5);
  vertical_wing.close();
  drivetrain.turn_toPID(173);
  drivetrain.drive_for(-56, -60, 1.2);
  vertical_wing.open();
  drivetrain.turn_toPID(155);
  brain.Screen.setCursor(10, 10);
  brain.Screen.print(brain.Timer.time(seconds) - t_i);
}

//fill in this function with stuff n things
void safe_defense(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake) {
  brain brain;
  double t_i = brain.Timer.time(seconds);
  drivetrain.set_initial_position(new double[2] {90, 45});
  drivetrain.Inertial.setHeading(45, degrees);
  drivetrain.drive_to_point(new double[2] {80, 80}, -50, 1.2);
  drivetrain.turn_to_test(270);
  drivetrain.drive_for(-30, -80, 0.5);
  drivetrain.drive_for(30, 40, 0.5);
  drivetrain.turn_to_test(305);
  wing_delay();
  drivetrain.drive_to_point(new double[2] {100, 27}, 40, 1);
  drivetrain.turn_to_test(0);
  drivetrain.drive_for(50, 50, 0.9);
  drivetrain.drive_for(-30, -50, 0.5);
  vertical_wing.close();
  drivetrain.turn_toPID(173);
  drivetrain.drive_for(-56, -60, 1.2);
  vertical_wing.open();
  drivetrain.turn_toPID(155);
  brain.Screen.setCursor(10, 10);
  brain.Screen.print(brain.Timer.time(seconds) - t_i);
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

void auto_skills(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wings, motor intake, motor launcher, pneumatics climber) {
  resetLauncher(launcher);
  drivetrain.drive_for(5, 50, 0.2);
  drivetrain.drive_for(-10, -60, 0.5);
  brain brain;
  double t_i = brain.Timer.time(seconds);
  int i = 0;
  while(brain.Timer.time(seconds) - t_i < 40) {
    launcher.spinFor(180, degrees);
    wait(0.25, seconds);
    i++;
    if(i == 5) {
      drivetrain.drive_for(-5, -25, 0.2);
      i = 0;
    }
  }
  drivetrain.Inertial.calibrate();
  while(drivetrain.Inertial.isCalibrating()) {
    wait(0.01, seconds);
  }
  drivetrain.drive_for(-10, -30, 0.4);
  drivetrain.Inertial.setHeading(225, degrees);
  drivetrain.drive_for(100, 75, 1);
  drivetrain.turn_to_test(185);
  drivetrain.drive_for(40, 60, 0.5);
  drivetrain.turn_toPID(90);
  drivetrain.drive_for(150, 100, 2.5);
  drivetrain.Inertial.setHeading(270, degrees);
  drivetrain.turn_to_test(270);
  horizontal_wings.open();
  //sweep
  drivetrain.drive_for(-100, -100, 1);
  drivetrain.drive_for(100, 50, 0.5);
  drivetrain.drive_for(-100, -100, 1);
  drivetrain.drive_for(100, 50, 0.5);
  drivetrain.LeftSide.spin(forward);
  drivetrain.RightSide.spin(reverse);
  drivetrain.LeftSide.setVelocity(100, percent);
  drivetrain.RightSide.setVelocity(100, percent);
  wait(4, seconds);
  drivetrain.LeftSide.stop();
  drivetrain.RightSide.stop();
}
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