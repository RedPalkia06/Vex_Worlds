#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "vex.h"
using namespace vex;

class Launcher {
    private:
        bool is_launching = false;
        void apply_launcher_state();
        motor launcher_motor;
        
    public:
        void start();
        void stop();
        void toggle();
        void fire_for_rotations(int rotations);

        // Constructor
        Launcher(int32_t port, gearSetting gear_ratio, bool reverse ) : launcher_motor(port, gear_ratio, reverse) {}
};
#endif
