#include "pneumatic_wings.h"

void PneumaticWing::apply_wing_state() {
  if (wings_are_out) {
    solenoid.open();
  } else {
    solenoid.close();
  }
}

void PneumaticWing::wings_out() {
  wings_are_out = true;
  apply_wing_state();
}

void PneumaticWing::wings_in() {
  wings_are_out = false;
  apply_wing_state();
}

void PneumaticWing::wings_toggle() {
  wings_are_out = !wings_are_out;
  apply_wing_state();
}