#include "vex.h"
#include "drivercontrol.h"
using namespace vex;

//launch functions: multiple options, figure out which one is best
void launch(motor m) {
    m.spinFor(forward, 180, degrees);
}

void launchLoop(controller Controller, motor m) {
   while (1) {
    if(Controller.ButtonL1.pressing()) {
        m.spinFor(forward, 180, degrees);
    }
    wait(1, msec);
   }
}

//intake
void intake(motor m, distance d) {
    //code with distance sensor here
    if(!(d.objectDistance(inches) < 5)) {
        m.spin(forward);
    } else {
        m.stop();
    }
}

void input(motor m, distance d) {
    m.spin(reverse);
}
//wings
void openWings(pneumatics p) {
    p.open();
}

void closeWings(pneumatics p) {
    p.close();
}

//climber