#pragma once

#include <stdint.h>
#include "Config.h"
#include "SystemTypes.h"

/**
 * @class SafetySupervisor
 * @brief Continuous hardware supervisor monitoring sensor integrity,
 *        thermal limits, object proximity boundaries, and hardware interlocks.
 */
class SafetySupervisor {
public:
    SafetySupervisor();

    /**
     * @brief Initialize supervisor pins and timers.
     */
    void begin();

    /**
     * @brief Evaluate MagLev operating conditions for faults.
     * @param raw_sensor Current raw ADC reading
     * @param error Current tracking error
     * @param pwm_duty Current commanded PWM output
     * @param dt_ms Elapsed milliseconds
     * @return true if operating within safe envelope, false if fault tripped
     */
    bool checkMagLevSafety(uint16_t raw_sensor, float error, uint16_t pwm_duty, uint32_t dt_ms);

    /**
     * @brief Evaluate hardware interlock to guarantee mutual exclusion
     *        between rover traction motors and the high-current MagLev coil.
     */
    bool verifyInterlock(SystemMode requested_mode);

    /**
     * @brief Trip an emergency stop immediately.
     */
    void triggerEmergencyStop(FaultReason reason);

    /**
     * @brief Clear faults and attempt re-arming to SAFE_MODE.
     */
    void resetFaults();

    // Status queries
    bool hasFault() const { return _current_fault != FaultReason::NONE; }
    FaultReason getFaultReason() const { return _current_fault; }

private:
    FaultReason _current_fault;
    uint32_t    _high_pwm_accumulator_ms;
    uint16_t    _out_of_bounds_count;
};
