#include "launcher.h"

void Launcher::apply_launcher_state() {
  if (is_launching) {
    launcher_motor.spin(forward);
  } else {
    launcher_motor.stop();
  }
}

void Launcher::start() {
  is_launching = true;
  apply_launcher_state();
}

void Launcher::stop() {
  is_launching = false;
  apply_launcher_state();
}

void Launcher::toggle() {
  is_launching = !is_launching;
  apply_launcher_state();
}

void Launcher::fire_for_rotations(int rotations) {

}