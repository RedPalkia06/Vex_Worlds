#ifndef DRIVERCONTROL_H
#define DRIVERCONTROL_H
#include "vex.h"
using namespace vex;

void launchLoop(controller Controller, motor m);
void intake(motor m);
void input(motor m);
void stop_intake(motor m);
void open_wings(pneumatics p);
void close_wings(pneumatics p);
void climb(pneumatics p);
void climb_down(pneumatics p);
void wing_down(pneumatics p);
void wing_up(pneumatics p);
#endif