#include "vex.h"
#include "drivercontrol.h"
using namespace vex;

void launchLoop(controller Controller, motor m) {
    m.spinFor(forward, 90, degrees);
    while (1) {
    if(Controller.ButtonR1.pressing()) {
        m.spinFor(forward, 90, degrees);
        wait(200, msec);
        m.spinFor(forward, 90, degrees);
    }
    wait(1, msec);
   }
}

//intake
void intake(motor m) {
    m.spin(forward);
}

void input(motor m) {
    m.spin(reverse);
}

void stopIntake(motor m) {
    m.stop();
}

//wings
void openWings(pneumatics p) {
    p.open();
}

void closeWings(pneumatics p) {
    p.close();
}

void wingDown(pneumatics p) {
    p.close();
}

void wingUp(pneumatics p) {
    p.open();
}

//climber

void climb(pneumatics p) {
    p.close();
}

void climbDown(pneumatics p) {
    p.open();
}