#include "SafetySupervisor.h"
#include <math.h>

#ifdef ARDUINO
    #include <Arduino.h>
#else
    static void digitalWrite(uint8_t pin, uint8_t val) { (void)pin; (void)val; }
    static void pinMode(uint8_t pin, uint8_t mode) { (void)pin; (void)mode; }
    #define HIGH 1
    #define LOW 0
    #define OUTPUT 1
#endif

SafetySupervisor::SafetySupervisor()
    : _current_fault(FaultReason::NONE),
      _high_pwm_accumulator_ms(0),
      _out_of_bounds_count(0) {}

void SafetySupervisor::begin() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    resetFaults();
}

void SafetySupervisor::resetFaults() {
    _current_fault = FaultReason::NONE;
    _high_pwm_accumulator_ms = 0;
    _out_of_bounds_count = 0;
    digitalWrite(PIN_STATUS_LED, LOW);
}

void SafetySupervisor::triggerEmergencyStop(FaultReason reason) {
    _current_fault = reason;
    digitalWrite(PIN_STATUS_LED, HIGH);
}

bool SafetySupervisor::verifyInterlock(SystemMode requested_mode) {
    // Mode transitions must be clean and mutually exclusive
    if (requested_mode == SystemMode::ROVER_MODE) {
        // Electromagnet PWM MUST be completely zeroed and disabled
        return true;
    } else if (requested_mode == SystemMode::MAGLEV_MODE) {
        // Rover H-bridge PWMs MUST be completely zeroed and disabled
        return true;
    } else if (requested_mode == SystemMode::SAFE_MODE) {
        return true;
    }
    return false;
}

bool SafetySupervisor::checkMagLevSafety(uint16_t raw_sensor, float error, uint16_t pwm_duty, uint32_t dt_ms) {
    // 1. Check for physical sensor disconnection (open pin floating or pulled to ground)
    if (raw_sensor < SENSOR_RAW_MIN_VALID) {
        triggerEmergencyStop(FaultReason::SENSOR_DISCONNECTED);
        return false;
    }

    // 2. Check for sensor rail saturation (shorted to VCC or magnet pressed against Hall chip)
    if (raw_sensor > SENSOR_RAW_MAX_VALID) {
        triggerEmergencyStop(FaultReason::SENSOR_SATURATED);
        return false;
    }

    // 3. Check for object dropped or thrown out of measurable magnetic gradient
    if (fabsf(error) > CRITICAL_DISPLACEMENT_ERROR) {
        _out_of_bounds_count++;
        // If error persists beyond 15 control cycles (approx 30 ms), trip fault
        if (_out_of_bounds_count > 15) {
            triggerEmergencyStop(FaultReason::OBJECT_DROPPED);
            return false;
        }
    } else {
        _out_of_bounds_count = 0;
    }

    // 4. Thermal protection against continuous high-current coil saturation
    if (pwm_duty >= THERMAL_HIGH_PWM_THRESHOLD) {
        _high_pwm_accumulator_ms += dt_ms;
        if (_high_pwm_accumulator_ms >= THERMAL_TIMEOUT_MS) {
            triggerEmergencyStop(FaultReason::THERMAL_OVERHEAT_TIMEOUT);
            return false;
        }
    } else {
        // Slowly decay thermal accumulator when operating at moderate duty cycle
        if (_high_pwm_accumulator_ms >= dt_ms) {
            _high_pwm_accumulator_ms -= dt_ms;
        } else {
            _high_pwm_accumulator_ms = 0;
        }
    }

    return true;
}
