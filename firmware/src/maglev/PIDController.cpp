#include "PIDController.h"

PIDController::PIDController()
    : _kp(DEFAULT_KP),
      _ki(DEFAULT_KI),
      _kd(DEFAULT_KD),
      _min_output(MAGLEV_MIN_PWM),
      _max_output(MAGLEV_MAX_PWM),
      _integral_min(PID_INTEGRAL_MIN),
      _integral_max(PID_INTEGRAL_MAX),
      _deriv_filter_alpha(PID_DERIVATIVE_FILTER_ALPHA),
      _integral_sum(0.0f),
      _last_error(0.0f),
      _filtered_derivative(0.0f),
      _is_first_run(true),
      _p_term(0.0f),
      _i_term(0.0f),
      _d_term(0.0f) {}

void PIDController::init(float kp, float ki, float kd,
                         float min_out, float max_out,
                         float integral_min, float integral_max,
                         float deriv_filter_alpha) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _min_output = min_out;
    _max_output = max_out;
    _integral_min = integral_min;
    _integral_max = integral_max;
    _deriv_filter_alpha = deriv_filter_alpha;
    reset();
}

void PIDController::reset() {
    _integral_sum = 0.0f;
    _last_error = 0.0f;
    _filtered_derivative = 0.0f;
    _is_first_run = true;
    _p_term = 0.0f;
    _i_term = 0.0f;
    _d_term = 0.0f;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    if (kp >= 0.0f) _kp = kp;
    if (ki >= 0.0f) _ki = ki;
    if (kd >= 0.0f) _kd = kd;
}

void PIDController::setOutputLimits(float min_out, float max_out) {
    if (min_out <= max_out) {
        _min_output = min_out;
        _max_output = max_out;
    }
}

float PIDController::compute(float target, float measured, float dt_seconds) {
    if (dt_seconds <= 0.0f) {
        return _min_output;
    }

    // In magnetic levitation:
    // If the object falls away, distance increases or magnetic flux decreases.
    // Error convention: error = target - measured
    float error = target - measured;

    // 1. Proportional Term
    _p_term = _kp * error;

    // 2. Integral Term with Anti-Windup Clamping
    // Accumulate error only if system is not severely saturated
    _integral_sum += error * dt_seconds;
    if (_integral_sum > _integral_max) {
        _integral_sum = _integral_max;
    } else if (_integral_sum < _integral_min) {
        _integral_sum = _integral_min;
    }
    _i_term = _ki * _integral_sum;

    // 3. Derivative Term with 1st-Order Low-Pass Filter
    // Raw derivative = rate of change of error
    float raw_derivative = 0.0f;
    if (!_is_first_run) {
        raw_derivative = (error - _last_error) / dt_seconds;
        // Low-pass exponential filter to mitigate ADC quantization and noise amplification
        _filtered_derivative = (_deriv_filter_alpha * raw_derivative) + 
                               ((1.0f - _deriv_filter_alpha) * _filtered_derivative);
    } else {
        _filtered_derivative = 0.0f;
        _is_first_run = false;
    }
    _d_term = _kd * _filtered_derivative;
    _last_error = error;

    // Total raw control action
    float raw_output = _p_term + _i_term + _d_term;

    // 4. Output Saturation & Clamping
    float clamped_output = raw_output;
    if (clamped_output > _max_output) {
        clamped_output = _max_output;
    } else if (clamped_output < _min_output) {
        clamped_output = _min_output;
    }

    return clamped_output;
}
