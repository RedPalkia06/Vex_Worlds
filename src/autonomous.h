#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
#include "vex.h"
#include "drivetrain.h"
#include "constants.h"
#include "math_utils.h"
#include "drivercontrol.h"
#include "pneumatic_wings.h"
#include <iostream>

void six_piece_auto(Drivetrain drivetrain, PneumaticWing side_wing, PneumaticWing horizontal_wing, motor intake);

#endif