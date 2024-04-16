#include "climber.h"

void Climber::apply_climber_state() {
  if (climber_is_up) {
    solenoid.open();
  } else {
    solenoid.close();
  }
}

void Climber::up() {
  climber_is_up = true;
  apply_climber_state();
}

void Climber::down() {
  climber_is_up = false;
  apply_climber_state();
}

void Climber::toggle() {
  climber_is_up = !climber_is_up;
  apply_climber_state();
}