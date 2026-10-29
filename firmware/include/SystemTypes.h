#pragma once

#include <stdint.h>

/**
 * @file SystemTypes.h
 * @brief Common state enumerations, fault codes, and telemetry structures.
 */

enum class SystemMode : uint8_t {
    SAFE_MODE = 0,    // Default boot state: all actuators disabled
    ROVER_MODE = 1,   // Rover motors active, MagLev coil strictly disabled
    MAGLEV_MODE = 2   // MagLev closed-loop active, Rover motors strictly disabled
};

inline const char* systemModeToString(SystemMode mode) {
    switch (mode) {
        case SystemMode::SAFE_MODE:   return "SAFE_MODE";
        case SystemMode::ROVER_MODE:  return "ROVER_MODE";
        case SystemMode::MAGLEV_MODE: return "MAGLEV_MODE";
        default:                      return "UNKNOWN";
    }
}

enum class MagLevState : uint8_t {
    IDLE = 0,
    CALIBRATING = 1,
    STABILIZING = 2,
    LEVITATING = 3,
    FAULT = 4,
    EMERGENCY_STOP = 5
};

inline const char* magLevStateToString(MagLevState state) {
    switch (state) {
        case MagLevState::IDLE:           return "IDLE";
        case MagLevState::CALIBRATING:    return "CALIBRATING";
        case MagLevState::STABILIZING:    return "STABILIZING";
        case MagLevState::LEVITATING:     return "LEVITATING";
        case MagLevState::FAULT:          return "FAULT";
        case MagLevState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                          return "UNKNOWN";
    }
}

enum class FaultReason : uint8_t {
    NONE = 0,
    SENSOR_DISCONNECTED = 1,
    SENSOR_SATURATED = 2,
    OBJECT_DROPPED = 3,
    THERMAL_OVERHEAT_TIMEOUT = 4,
    HARDWARE_INTERLOCK_VIOLATION = 5,
    INVALID_COMMAND = 6
};

inline const char* faultReasonToString(FaultReason reason) {
    switch (reason) {
        case FaultReason::NONE:                         return "OK";
        case FaultReason::SENSOR_DISCONNECTED:          return "SENSOR_DISCONNECTED";
        case FaultReason::SENSOR_SATURATED:             return "SENSOR_SATURATED";
        case FaultReason::OBJECT_DROPPED:               return "OBJECT_OUT_OF_BOUNDS";
        case FaultReason::THERMAL_OVERHEAT_TIMEOUT:     return "THERMAL_OVERHEAT_TIMEOUT";
        case FaultReason::HARDWARE_INTERLOCK_VIOLATION: return "INTERLOCK_VIOLATION";
        case FaultReason::INVALID_COMMAND:              return "INVALID_COMMAND";
        default:                                        return "UNKNOWN_FAULT";
    }
}

/**
 * @brief Telemetry snapshot for magnetic levitation closed-loop state.
 */
struct MagLevTelemetry {
    uint32_t timestamp_ms;
    SystemMode current_mode;
    MagLevState control_state;
    FaultReason fault_code;
    
    uint16_t raw_sensor;
    float baseline_offset;
    float filtered_position;
    float target_position;
    float error;
    
    float p_term;
    float i_term;
    float d_term;
    float pid_output;
    uint16_t pwm_duty;
};

/**
 * @brief Telemetry snapshot for rover mobile subsystem.
 */
struct RoverTelemetry {
    uint32_t timestamp_ms;
    SystemMode current_mode;
    float ultrasonic_distance_cm;
    bool ir_left_blocked;
    bool ir_right_blocked;
    int16_t left_motor_speed;
    int16_t right_motor_speed;
    const char* rover_status;
};
