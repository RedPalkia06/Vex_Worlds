#ifndef CLIMBER_H
#define CLIMBER_H

#include "vex.h"
using namespace vex;

class Climber {
    private:
        bool climber_is_up = false;
        void apply_climber_state();
        pneumatics solenoid;
        
    public:
        void up();
        void down();
        void toggle();

        // Constructor
        Climber(vex::triport::port port) : solenoid(port) {
        }
};
#endif
