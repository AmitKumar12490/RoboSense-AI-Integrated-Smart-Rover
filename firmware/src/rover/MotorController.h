#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * @class MotorController
 * @brief Differential drive motor controller for L298N / TB6612FNG H-bridge drivers.
 */
class MotorController {
public:
    MotorController();

    /**
     * @brief Configure directional GPIO pins and ESP32 LEDC PWM channels.
     */
    void begin();

    /**
     * @brief Drive both motors with differential speeds (-255 to +255).
     * @param left_speed Speed and direction for left motor
     * @param right_speed Speed and direction for right motor
     */
    void drive(int16_t left_speed, int16_t right_speed);

    /**
     * @brief Predefined directional maneuvers.
     */
    void forward(uint8_t speed = 180);
    void backward(uint8_t speed = 160);
    void turnLeft(uint8_t speed = 160);
    void turnRight(uint8_t speed = 160);
    void stop();

    int16_t getLeftSpeed() const { return _current_left; }
    int16_t getRightSpeed() const { return _current_right; }

private:
    void setSingleMotor(uint8_t pwm_ch, uint8_t pin_in1, uint8_t pin_in2, int16_t speed);

    int16_t _current_left;
    int16_t _current_right;
};
