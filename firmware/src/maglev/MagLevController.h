#pragma once

#include <stdint.h>
#include "Config.h"
#include "SystemTypes.h"
#include "PIDController.h"
#include "SensorCalibration.h"
#include "safety/SafetySupervisor.h"

/**
 * @class MagLevController
 * @brief High-frequency electromagnetic levitation feedback controller.
 *
 * Coordinates ADC sensor sampling, PID computation, MOSFET PWM driving,
 * and safety monitoring within a deterministic loop.
 */
class MagLevController {
public:
    MagLevController();

    /**
     * @brief Configure GPIOs, hardware LEDC timer, and calibrate sensor.
     */
    void begin(SafetySupervisor* supervisor);

    /**
     * @brief High-speed deterministic execution step.
     * @param dt_us Microseconds elapsed since last step
     */
    void update(uint32_t dt_us);

    /**
     * @brief Start active levitation closed-loop control.
     * @return true if armed successfully, false if fault prevents arming
     */
    bool start();

    /**
     * @brief Disable electromagnet PWM immediately and return to IDLE.
     */
    void stop();

    /**
     * @brief Trigger baseline calibration tare.
     */
    bool calibrate();

    /**
     * @brief Update target setpoint position safely.
     */
    void setTarget(float target);

    /**
     * @brief Update PID gain parameters safely with bounds checking.
     */
    bool setTunings(float kp, float ki, float kd);

    // State & telemetry accessors
    MagLevState getState() const { return _state; }
    MagLevTelemetry getTelemetry(SystemMode current_mode) const;
    uint16_t getCurrentPWM() const { return _current_pwm; }

private:
    void applyPWM(uint16_t duty);
    void forceSafeCutoff();

    MagLevState        _state;
    PIDController      _pid;
    SensorCalibration  _sensor;
    SafetySupervisor*  _supervisor;

    float              _target_position;
    uint16_t           _current_pwm;
    uint16_t           _last_raw_sensor;
    float              _last_position;
    float              _last_error;
};
