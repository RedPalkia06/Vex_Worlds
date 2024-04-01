#ifndef DRIVERCONTROL_H
#define DRIVERCONTROL_H
#include "vex.h"
using namespace vex;

void launch(motor m);
void launchLoop(controller Controller, motor m);
void intake(motor m, distance d);
void input(motor m, distance d);
void openWings(pneumatics p);
void closeWings(pneumatics p);

#endif