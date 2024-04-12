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
void climb(pneumatics p);
void climbDown(pneumatics p);
void wingDown(pneumatics p);
void wingUp(pneumatics p);
#endif