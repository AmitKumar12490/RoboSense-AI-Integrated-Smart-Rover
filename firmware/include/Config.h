#pragma once

#include <stdint.h>

/**
 * @file Config.h
 * @brief Pin definitions, control constants, and safety thresholds for RoboSense.
 *
 * All hardware parameters are centralized here to maintain clear electrical
 * and timing constraints across the rover and electromagnetic levitation subsystems.
 */

// ============================================================================
// OPERATING FREQUENCY & TIMING CONSTANTS
// ============================================================================
// Fixed deterministic control interval in microseconds (2000 us = 2.0 ms = 500 Hz)
constexpr uint32_t MAGLEV_CONTROL_INTERVAL_US = 2000;

// Telemetry broadcast interval in milliseconds (50 ms = 20 Hz)
constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;

// Rover obstacle avoidance evaluation interval in milliseconds (50 ms = 20 Hz)
constexpr uint32_t ROVER_CONTROL_INTERVAL_MS = 50;

// ============================================================================
// MAGLEV PIN DEFINITIONS & PWM SETTINGS
// ============================================================================
// GPIO 23: Hardware LEDC PWM output to logic-level N-channel MOSFET gate
constexpr uint8_t PIN_MAGLEV_PWM = 23;

// GPIO 34: ADC1 Input for Linear Hall-Effect Sensor (SS495A / A1302 via divider)
// NOTE: ADC1 is selected because ADC2 cannot be used concurrently with ESP32 Wi-Fi.
constexpr uint8_t PIN_MAGLEV_SENSOR_ADC = 34;

// GPIO 2: Built-in status / heartbeat / emergency LED
constexpr uint8_t PIN_STATUS_LED = 2;

// PWM Configuration:
// 25 kHz carrier frequency: Well beyond the human audible range (prevents acoustic whine),
// while low enough for standard logic MOSFET gate capacitances without excessive switching losses.
constexpr uint32_t MAGLEV_PWM_FREQ_HZ = 25000;
constexpr uint8_t  MAGLEV_PWM_CHANNEL = 0;
constexpr uint8_t  MAGLEV_PWM_RESOLUTION_BITS = 10; // 0 to 1023 (10-bit)
constexpr uint16_t MAGLEV_PWM_MAX_RANGE = 1023;

// ============================================================================
// MAGLEV CONTROL & PID DEFAULT PARAMETERS
// ============================================================================
// Starting PID constants (clearly marked as initial experimental baseline, requires tuning):
constexpr float DEFAULT_KP = 1.85f;
constexpr float DEFAULT_KI = 0.08f;
constexpr float DEFAULT_KD = 0.42f;

// Setpoint position (relative ADC units centered around calibrated equilibrium):
constexpr float DEFAULT_TARGET_POSITION = 512.0f;

// Duty cycle saturation limits:
constexpr uint16_t MAGLEV_MIN_PWM = 0;
// Maximum allowed PWM duty cycle clamped at ~85% (870 / 1023) to avoid coil thermal saturation
// and continuous full-rail inductive current draw.
constexpr uint16_t MAGLEV_MAX_PWM = 870;

// Integral anti-windup accumulator clamp
constexpr float PID_INTEGRAL_MAX = 400.0f;
constexpr float PID_INTEGRAL_MIN = -400.0f;

// Low-pass filter coefficient for derivative action to suppress ADC high-frequency noise
// Derivative D_filtered = alpha * D_raw + (1 - alpha) * D_prev
constexpr float PID_DERIVATIVE_FILTER_ALPHA = 0.25f;

// Exponential moving average filter factor for sensor input
constexpr float SENSOR_EMA_ALPHA = 0.30f;

// Sensor calibration sample count
constexpr uint16_t SENSOR_CALIBRATION_SAMPLES = 250;

// ============================================================================
// ELECTRICAL & MECHANICAL SAFETY THRESHOLDS
// ============================================================================
// 12-bit ADC raw range is 0 - 4095. Values below 60 or above 4030 indicate disconnected
// sensor wires or rail short-circuits.
constexpr uint16_t SENSOR_RAW_MIN_VALID = 60;
constexpr uint16_t SENSOR_RAW_MAX_VALID = 4035;

// Maximum acceptable error displacement before assuming object has dropped or crashed
constexpr float CRITICAL_DISPLACEMENT_ERROR = 620.0f;

// Overcurrent / continuous high-power timeout:
// If PWM remains above 800 continuously for longer than 3.5 seconds, shut down to prevent thermal runaway
constexpr uint16_t THERMAL_HIGH_PWM_THRESHOLD = 800;
constexpr uint32_t THERMAL_TIMEOUT_MS = 3500;

// ============================================================================
// ROVER PIN DEFINITIONS & PWM SETTINGS
// ============================================================================
// Motor Driver Pins (L298N / TB6612FNG Dual H-Bridge)
constexpr uint8_t PIN_MOTOR_LEFT_PWM   = 25;
constexpr uint8_t PIN_MOTOR_LEFT_DIR1  = 26;
constexpr uint8_t PIN_MOTOR_LEFT_DIR2  = 27;

constexpr uint8_t PIN_MOTOR_RIGHT_PWM  = 14;
constexpr uint8_t PIN_MOTOR_RIGHT_DIR1 = 12;
constexpr uint8_t PIN_MOTOR_RIGHT_DIR2 = 13;

// Rover Motor PWM Channels (8-bit resolution: 0 - 255)
constexpr uint8_t  ROVER_PWM_LEFT_CHANNEL  = 2;
constexpr uint8_t  ROVER_PWM_RIGHT_CHANNEL = 3;
constexpr uint32_t ROVER_PWM_FREQ_HZ       = 5000;
constexpr uint8_t  ROVER_PWM_RES_BITS      = 8;

// Rover Obstacle Detection Pins
constexpr uint8_t PIN_ULTRASONIC_TRIG  = 5;
constexpr uint8_t PIN_ULTRASONIC_ECHO  = 18;
constexpr uint8_t PIN_IR_OBSTACLE_LEFT = 19;
constexpr uint8_t PIN_IR_OBSTACLE_RIGHT = 21;

// Distance thresholds in centimeters
constexpr float ROVER_OBSTACLE_STOP_CM = 18.0f;
constexpr float ROVER_OBSTACLE_WARN_CM = 35.0f;
