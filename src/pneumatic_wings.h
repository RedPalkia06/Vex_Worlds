#ifndef PNEUMATIC_WINGS_H
#define PNEUMATIC_WINGS_H

#include "vex.h"
using namespace vex;

class PneumaticWing {
    private:
        bool wings_are_out = false;
        void apply_wing_state();
        pneumatics solenoid;
        
    public:
        void wings_out();
        void wings_in();
        void wings_toggle();

        // Constructor
        PneumaticWing(vex::triport::port port) : solenoid(port) {
        }
};
#endif
