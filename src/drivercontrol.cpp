#include "vex.h"
#include "drivercontrol.h"
using namespace vex;

void endLaunching(motor m) {
    if(m.isSpinning()) {
        wait(1, seconds);
        if(!m.isSpinning()) {
            m.setStopping(coast);
            return;
        }
    }
}

void launchLoop(controller Controller, motor m) {
    while (1) {
        m.setStopping(hold);
        if(Controller.ButtonR1.pressing()) {
            m.spinFor(forward, 90, degrees);
            wait(250, msec);
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

void stop_intake(motor m) {
    m.stop();
}


//climber

void climb(pneumatics p) {
    p.close();
}

void climb_down(pneumatics p) {
    p.open();
}

void resetLauncher(motor &m) {
    m.spinFor(forward, 70, degrees);
    m.setStopping(hold);
}