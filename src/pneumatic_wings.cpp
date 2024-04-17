#include "pneumatic_wings.h"

void PneumaticWing::apply_wing_state() {
  if (wings_are_out) {
    this->solenoid.open();
  } else {
    this->solenoid.close();
  }
}

void PneumaticWing::out() {
  this->wings_are_out = true;
  this->apply_wing_state();
}

void PneumaticWing::up() {
  this->out();
}

void PneumaticWing::in() {
  this->wings_are_out = false;
  this->apply_wing_state();
}

void PneumaticWing::down() {
  this->in();
}

void PneumaticWing::toggle() {
  this->wings_are_out = !this->wings_are_out;
  this->apply_wing_state();
}