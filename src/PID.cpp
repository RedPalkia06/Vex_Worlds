#include "PID.h"
#include "vex.h"
using namespace vex;

void PIDController::set_tunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::reset() {
    _error_integral = 0;
    _control_output = 0;
    _previous_error = setpoint - _current_value;
}

float PIDController::update(float current_value) {
    float current_time = _brain.Timer.time(seconds);
    float delta_time = current_time - _previous_time;

    if (delta_time < time_step)
        return _control_output;

    _previous_time = current_time;

    float current_error = setpoint - current_value;
    _error_integral += current_error * delta_time;

    _last_error_derivative = (current_error - _previous_error) / delta_time;

    _control_output = kp * current_error + ki * _error_integral + kd * _last_error_derivative;
    _previous_error = current_error;

    return _control_output;
}

bool PIDController::at_setpoint() {
    return _previous_error <= position_tolerance && _last_error_derivative < velocity_tolerance;
}