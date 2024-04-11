#ifndef DRIVERCONTROL_H
#define DRIVERCONTROL_H
#include "vex.h"
using namespace vex;

void launchLoop(controller Controller, motor m);
void intake(motor m);
void input(motor m);
void stopIntake(motor m);
void openWings(pneumatics p);
void closeWings(pneumatics p);

#endif