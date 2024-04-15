#ifndef PID_H
#define PID_H

#include "vex.h"

using namespace vex;
class PIDController {
public:
    /**
     * A generalized PID controller implementation.
     */

    /**
     * Initializes a PIDController instance.
     * @param timer The timer object used to measure time.
     * @param kp Kp value for the PID.
     * @param ki Ki value for the PID.
     * @param kd Kd value for the PID.
     * @param t Minimum time between update calls.
     *        All calls made before this amount of time has passed since the last calculation will be ignored.
     * @param integral_zone The lower and upper bounds for the integral term to prevent windup.
     */
    PIDController(brain brain, float kp = 1.0, float ki = 0.0, float kd = 0.0, float t = 0.05) :
            kp(kp),
            ki(ki),
            kd(kd),
            time_step(t),
            position_tolerance(0),
            velocity_tolerance(0),
            setpoint(0.0),
            _brain(brain),
            _previous_time(brain.Timer.time(seconds)),
             _current_value(0.0),
             _error_integral(0.0),
            _last_error_derivative(0.0),
            _previous_error(0.0),
            _control_output(0.0) {}

    /**
     * Sets the tuning constants (Kp, Ki, Kd) of the PID controller.
     * @param kp The proportional constant.
     * @param ki The integral constant.
     * @param kd The derivative constant.
     */
    void set_tunings(float kp, float ki, float kd);

    /**
     * Resets the PID controller's internal state.
     */
    void reset();

    /**
     * Updates the PID controller with the current measurement and returns the control output.
     * @param current_value The current measurement or feedback value.
     * @return The calculated control output.
     */
    float update(float current_value);

    /**
     * Checks if the PID controller has reached its setpoint.
     * @return True if at setpoint, False otherwise.
     */
    bool at_setpoint();

private:
    float kp; /**< Kp value for the PID. */
    float ki; /**< Ki value for the PID. */
    float kd; /**< Kd value for the PID. */
    float time_step; /**< Minimum time between update calls. */
    float position_tolerance; /**< Tolerance for position. */
    float velocity_tolerance; /**< Tolerance for velocity. */
    float setpoint; /**< The desired setpoint. */
    brain _brain; /**< The brain object used to measure time. */
    float _previous_time; /**< Previous time of update. */
    float _current_value; /**< Current value. */
    float _error_integral; /**< Integral of the error. */
    float _last_error_derivative; /**< Derivative of the error. */
    float _previous_error; /**< Previous error. */
    float _control_output; /**< Control output. */
};

#endif