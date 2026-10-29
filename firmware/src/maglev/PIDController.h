#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * @class PIDController
 * @brief Industrial-grade discrete PID controller with anti-windup,
 *        derivative low-pass filtering, and output saturation limits.
 *
 * Specifically tailored for magnetic levitation dynamics (unstable equilibrium,
 * fast pole, and non-linear electromagnetic force inverse to distance squared).
 */
class PIDController {
public:
    PIDController();

    /**
     * @brief Initialize or reconfigure PID gains and limits.
     */
    void init(float kp = DEFAULT_KP, float ki = DEFAULT_KI, float kd = DEFAULT_KD,
              float min_out = MAGLEV_MIN_PWM, float max_out = MAGLEV_MAX_PWM,
              float integral_min = PID_INTEGRAL_MIN, float integral_max = PID_INTEGRAL_MAX,
              float deriv_filter_alpha = PID_DERIVATIVE_FILTER_ALPHA);

    /**
     * @brief Compute the control signal given target setpoint, measured position, and time step dt.
     * @param target Desired target position
     * @param measured Current filtered position
     * @param dt_seconds Elapsed time in seconds since previous computation
     * @return Clamped control output (PWM duty cycle)
     */
    float compute(float target, float measured, float dt_seconds);

    /**
     * @brief Reset internal integral accumulator and derivative memory.
     * Must be called on state changes to prevent transient spikes (bumpless start).
     */
    void reset();

    // Gain adjustment methods (strictly verified within safe bounds)
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min_out, float max_out);

    // Diagnostics & terms getters
    float getPTerm() const { return _p_term; }
    float getITerm() const { return _i_term; }
    float getDTerm() const { return _d_term; }
    float getLastError() const { return _last_error; }
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

private:
    float _kp;
    float _ki;
    float _kd;

    float _min_output;
    float _max_output;
    float _integral_min;
    float _integral_max;
    float _deriv_filter_alpha;

    float _integral_sum;
    float _last_error;
    float _filtered_derivative;
    bool  _is_first_run;

    // Diagnostic breakdown of current control effort
    float _p_term;
    float _i_term;
    float _d_term;
};
