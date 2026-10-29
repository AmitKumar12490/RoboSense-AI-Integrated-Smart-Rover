#include "RoverSubsystem.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    static uint32_t millis() { return 0; }
#endif

RoverSubsystem::RoverSubsystem()
    : _state(RoverState::IDLE),
      _last_distance_cm(100.0f),
      _ir_left(false),
      _ir_right(false),
      _state_timer_ms(0),
      _turn_direction_left(true) {}

void RoverSubsystem::begin() {
    _motors.begin();
    _sensors.begin();
    stop();
}

void RoverSubsystem::stop() {
    _motors.stop();
    _state = RoverState::IDLE;
}

void RoverSubsystem::enableAutonomousMode() {
    _state = RoverState::CRUISING_FORWARD;
    _motors.forward(170);
}

void RoverSubsystem::setManualDrive(int16_t left_speed, int16_t right_speed) {
    _state = RoverState::MANUAL_CONTROL;
    _motors.drive(left_speed, right_speed);
}

void RoverSubsystem::update(uint32_t dt_ms) {
    _last_distance_cm = _sensors.readUltrasonicDistanceCm();
    _ir_left = _sensors.isLeftBlocked();
    _ir_right = _sensors.isRightBlocked();

    if (_state_timer_ms > 0) {
        if (_state_timer_ms >= dt_ms) {
            _state_timer_ms -= dt_ms;
        } else {
            _state_timer_ms = 0;
        }
    }

    if (_state == RoverState::CRUISING_FORWARD ||
        _state == RoverState::AVOIDING_OBSTACLE_BACKWARD ||
        _state == RoverState::TURNING_AWAY) {
        stepAutonomousFSM();
    }
}

void RoverSubsystem::stepAutonomousFSM() {
    bool obstacle_ahead = (_last_distance_cm <= ROVER_OBSTACLE_STOP_CM) || _ir_left || _ir_right;

    switch (_state) {
        case RoverState::CRUISING_FORWARD:
            if (obstacle_ahead) {
                // Obstacle encountered: immediately stop and initiate back-up maneuver
                _motors.stop();
                _turn_direction_left = _ir_right || (!_ir_left && (_last_distance_cm < 12.0f));
                _state = RoverState::AVOIDING_OBSTACLE_BACKWARD;
                _state_timer_ms = 350; // Back up for 350ms
                _motors.backward(160);
            } else {
                _motors.forward(170);
            }
            break;

        case RoverState::AVOIDING_OBSTACLE_BACKWARD:
            if (_state_timer_ms == 0) {
                _motors.stop();
                _state = RoverState::TURNING_AWAY;
                _state_timer_ms = 450; // Spin turn for 450ms
                if (_turn_direction_left) {
                    _motors.turnLeft(175);
                } else {
                    _motors.turnRight(175);
                }
            }
            break;

        case RoverState::TURNING_AWAY:
            if (_state_timer_ms == 0) {
                _motors.stop();
                // Check if forward path is now clear
                if (_last_distance_cm > ROVER_OBSTACLE_STOP_CM && !_ir_left && !_ir_right) {
                    _state = RoverState::CRUISING_FORWARD;
                    _motors.forward(170);
                } else {
                    // Still blocked: spin further
                    _state_timer_ms = 300;
                    if (_turn_direction_left) {
                        _motors.turnLeft(175);
                    } else {
                        _motors.turnRight(175);
                    }
                }
            }
            break;

        default:
            break;
    }
}

RoverTelemetry RoverSubsystem::getTelemetry(SystemMode current_mode) {
    RoverTelemetry t;
    t.timestamp_ms = millis();
    t.current_mode = current_mode;
    t.ultrasonic_distance_cm = _last_distance_cm;
    t.ir_left_blocked = _ir_left;
    t.ir_right_blocked = _ir_right;
    t.left_motor_speed = _motors.getLeftSpeed();
    t.right_motor_speed = _motors.getRightSpeed();

    switch (_state) {
        case RoverState::IDLE:                       t.rover_status = "IDLE"; break;
        case RoverState::CRUISING_FORWARD:           t.rover_status = "CRUISING_FORWARD"; break;
        case RoverState::AVOIDING_OBSTACLE_BACKWARD: t.rover_status = "BACKING_AWAY"; break;
        case RoverState::TURNING_AWAY:               t.rover_status = "TURNING_AWAY"; break;
        case RoverState::MANUAL_CONTROL:             t.rover_status = "MANUAL_DRIVE"; break;
        case RoverState::SAFETY_LOCKED:              t.rover_status = "SAFETY_LOCKED"; break;
        default:                                     t.rover_status = "UNKNOWN"; break;
    }

    return t;
}
