#include "MagLevController.h"

#ifdef ARDUINO
    #include <Arduino.h>
    #include <esp32-hal-ledc.h>
#else
    static uint32_t millis() { return 0; }
    static void ledcSetup(uint8_t ch, uint32_t freq, uint8_t res) { (void)ch; (void)freq; (void)res; }
    static void ledcAttachPin(uint8_t pin, uint8_t ch) { (void)pin; (void)ch; }
    static void ledcWrite(uint8_t ch, uint32_t duty) { (void)ch; (void)duty; }
#endif

MagLevController::MagLevController()
    : _state(MagLevState::IDLE),
      _supervisor(nullptr),
      _target_position(DEFAULT_TARGET_POSITION),
      _current_pwm(0),
      _last_raw_sensor(2048),
      _last_position(0.0f),
      _last_error(0.0f) {}

void MagLevController::begin(SafetySupervisor* supervisor) {
    _supervisor = supervisor;

#ifdef ARDUINO
    // Configure ESP32 hardware LEDC PWM for gate control of N-channel MOSFET
    ledcSetup(MAGLEV_PWM_CHANNEL, MAGLEV_PWM_FREQ_HZ, MAGLEV_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MAGLEV_PWM, MAGLEV_PWM_CHANNEL);
    ledcWrite(MAGLEV_PWM_CHANNEL, 0);
#endif

    _sensor.begin(PIN_MAGLEV_SENSOR_ADC);
    _pid.init(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
              MAGLEV_MIN_PWM, MAGLEV_MAX_PWM,
              PID_INTEGRAL_MIN, PID_INTEGRAL_MAX,
              PID_DERIVATIVE_FILTER_ALPHA);

    forceSafeCutoff();
    _state = MagLevState::IDLE;
}

void MagLevController::applyPWM(uint16_t duty) {
    if (duty > MAGLEV_MAX_PWM) {
        duty = MAGLEV_MAX_PWM;
    }
    _current_pwm = duty;
#ifdef ARDUINO
    ledcWrite(MAGLEV_PWM_CHANNEL, _current_pwm);
#endif
}

void MagLevController::forceSafeCutoff() {
    _current_pwm = 0;
#ifdef ARDUINO
    ledcWrite(MAGLEV_PWM_CHANNEL, 0);
#endif
    _pid.reset();
}

bool MagLevController::calibrate() {
    forceSafeCutoff();
    _state = MagLevState::CALIBRATING;

    bool success = _sensor.calibrateBaseline(SENSOR_CALIBRATION_SAMPLES);
    if (success) {
        _state = MagLevState::IDLE;
    } else {
        _state = MagLevState::FAULT;
        if (_supervisor) {
            _supervisor->triggerEmergencyStop(FaultReason::SENSOR_DISCONNECTED);
        }
    }
    return success;
}

bool MagLevController::start() {
    if (_supervisor && _supervisor->hasFault()) {
        return false;
    }

    if (!_sensor.isCalibrated()) {
        if (!calibrate()) {
            return false;
        }
    }

    _pid.reset();
    _state = MagLevState::LEVITATING;
    return true;
}

void MagLevController::stop() {
    forceSafeCutoff();
    _state = MagLevState::IDLE;
}

void MagLevController::setTarget(float target) {
    // Target position is bounded within measurable linear range of Hall sensor
    if (target >= -1500.0f && target <= 1500.0f) {
        _target_position = target;
    }
}

bool MagLevController::setTunings(float kp, float ki, float kd) {
    // Reject negative or dangerous outlier gains
    if (kp >= 0.0f && kp <= 25.0f &&
        ki >= 0.0f && ki <= 10.0f &&
        kd >= 0.0f && kd <= 10.0f) {
        _pid.setTunings(kp, ki, kd);
        return true;
    }
    return false;
}

void MagLevController::update(uint32_t dt_us) {
    // 1. Read hardware analog input from linear Hall sensor
    _last_raw_sensor = _sensor.readRawHardware();

    // 2. Process reading through calibration offset and exponential low-pass filter
    _last_position = _sensor.update(_last_raw_sensor);
    _last_error = _target_position - _last_position;

    // Convert elapsed microsecond delta to floating-point seconds and milliseconds
    float dt_seconds = static_cast<float>(dt_us) / 1000000.0f;
    uint32_t dt_ms = dt_us / 1000;
    if (dt_ms == 0) dt_ms = 1;

    // 3. Evaluate safety supervisor
    if (_supervisor) {
        bool is_safe = _supervisor->checkMagLevSafety(_last_raw_sensor, _last_error, _current_pwm, dt_ms);
        if (!is_safe) {
            forceSafeCutoff();
            _state = MagLevState::FAULT;
            return;
        }
    }

    // 4. Closed-loop state machine execution
    if (_state == MagLevState::LEVITATING) {
        // Compute discrete PID control effort
        float pid_out = _pid.compute(_target_position, _last_position, dt_seconds);

        // Convert PID control effort to integer PWM duty cycle
        uint16_t commanded_pwm = static_cast<uint16_t>(pid_out);
        applyPWM(commanded_pwm);
    } else {
        // In all non-levitating states (IDLE, CALIBRATING, FAULT, STOP), coil is strictly unpowered
        forceSafeCutoff();
    }
}

MagLevTelemetry MagLevController::getTelemetry(SystemMode current_mode) const {
    MagLevTelemetry t;
    t.timestamp_ms = millis();
    t.current_mode = current_mode;
    t.control_state = _state;
    t.fault_code = _supervisor ? _supervisor->getFaultReason() : FaultReason::NONE;

    t.raw_sensor = _last_raw_sensor;
    t.baseline_offset = _sensor.getBaseline();
    t.filtered_position = _last_position;
    t.target_position = _target_position;
    t.error = _last_error;

    t.p_term = _pid.getPTerm();
    t.i_term = _pid.getITerm();
    t.d_term = _pid.getDTerm();
    t.pid_output = t.p_term + t.i_term + t.d_term;
    t.pwm_duty = _current_pwm;

    return t;
}
