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
    PIDController(brain brain, double kp, double ki, double kd, double t, double position_tolerance_in, double velocity_tolerance_in) :
            kp(kp),
            ki(ki),
            kd(kd),
            time_step(t),
            position_tolerance(position_tolerance_in),
            velocity_tolerance(velocity_tolerance_in),
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
    void set_tunings(double kp, double ki, double kd);

    /**
     * Resets the PID controller's internal state.
     */
    void reset();

    /**
     * Updates the PID controller with the current measurement and returns the control output.
     * @param current_error The current measuremen's deviation from the target.
     * @return The calculated control output.
     */
    double update(double current_error);

    /**
     * Checks if the PID controller has reached its setpoint.
     * @return True if at setpoint, False otherwise.
     */
    bool at_setpoint();

    double setpoint; /**< The desired setpoint. */
    double _error_integral; /**< Integral of the error. */


private:
    double kp; /**< Kp value for the PID. */
    double ki; /**< Ki value for the PID. */
    double kd; /**< Kd value for the PID. */
    double time_step; /**< Minimum time between update calls. */
    double position_tolerance; /**< Tolerance for position. */
    double velocity_tolerance; /**< Tolerance for velocity. */
    brain _brain; /**< The brain object used to measure time. */
    double _previous_time; /**< Previous time of update. */
    double _current_value; /**< Current value. */
    double _last_error_derivative; /**< Derivative of the error. */
    double _previous_error; /**< Previous error. */
    double _control_output; /**< Control output. */
};

#endif