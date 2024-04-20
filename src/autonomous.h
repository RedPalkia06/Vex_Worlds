#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
#include "vex.h"
#include "drivetrain.h"
#include "constants.h"
#include "math_utils.h"
#include "drivercontrol.h"
#include "pneumatic_wings.h"
#include <iostream>

void six_piece(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake);
void five_piece(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake);
void max_defense(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake);
void safe_defense(Drivetrain &drivetrain, pneumatics vertical_wing, pneumatics horizontal_wing, motor intake);

#endif