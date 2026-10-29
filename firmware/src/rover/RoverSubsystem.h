#pragma once

#include <stdint.h>
#include "Config.h"
#include "SystemTypes.h"
#include "MotorController.h"
#include "ObstacleSensor.h"

enum class RoverState : uint8_t {
    IDLE = 0,
    CRUISING_FORWARD = 1,
    AVOIDING_OBSTACLE_BACKWARD = 2,
    TURNING_AWAY = 3,
    MANUAL_CONTROL = 4,
    SAFETY_LOCKED = 5
};

/**
 * @class RoverSubsystem
 * @brief High-level autonomous and manual navigation controller for the rover.
 */
class RoverSubsystem {
public:
    RoverSubsystem();

    void begin();
    void update(uint32_t dt_ms);

    void enableAutonomousMode();
    void setManualDrive(int16_t left_speed, int16_t right_speed);
    void stop();

    RoverState getState() const { return _state; }
    RoverTelemetry getTelemetry(SystemMode current_mode);

private:
    void stepAutonomousFSM();

    MotorController _motors;
    ObstacleSensor  _sensors;
    RoverState      _state;

    float           _last_distance_cm;
    bool            _ir_left;
    bool            _ir_right;
    uint32_t        _state_timer_ms;
    bool            _turn_direction_left;
};
