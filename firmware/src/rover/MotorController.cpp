#include "MotorController.h"
#include <stdlib.h>

#ifdef ARDUINO
    #include <Arduino.h>
    #include <esp32-hal-ledc.h>
#else
    static void pinMode(uint8_t pin, uint8_t mode) { (void)pin; (void)mode; }
    static void digitalWrite(uint8_t pin, uint8_t val) { (void)pin; (void)val; }
    static void ledcSetup(uint8_t ch, uint32_t freq, uint8_t res) { (void)ch; (void)freq; (void)res; }
    static void ledcAttachPin(uint8_t pin, uint8_t ch) { (void)pin; (void)ch; }
    static void ledcWrite(uint8_t ch, uint32_t duty) { (void)ch; (void)duty; }
    #define OUTPUT 1
    #define HIGH 1
    #define LOW 0
#endif

MotorController::MotorController()
    : _current_left(0),
      _current_right(0) {}

void MotorController::begin() {
    pinMode(PIN_MOTOR_LEFT_DIR1, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_DIR2, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_DIR2, OUTPUT);

#ifdef ARDUINO
    ledcSetup(ROVER_PWM_LEFT_CHANNEL, ROVER_PWM_FREQ_HZ, ROVER_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_LEFT_PWM, ROVER_PWM_LEFT_CHANNEL);

    ledcSetup(ROVER_PWM_RIGHT_CHANNEL, ROVER_PWM_FREQ_HZ, ROVER_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RIGHT_PWM, ROVER_PWM_RIGHT_CHANNEL);
#endif

    stop();
}

void MotorController::setSingleMotor(uint8_t pwm_ch, uint8_t pin_in1, uint8_t pin_in2, int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        // Forward rotation
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
#ifdef ARDUINO
        ledcWrite(pwm_ch, static_cast<uint32_t>(speed));
#endif
    } else if (speed < 0) {
        // Reverse rotation
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
#ifdef ARDUINO
        ledcWrite(pwm_ch, static_cast<uint32_t>(-speed));
#endif
    } else {
        // Active brake / coast
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
#ifdef ARDUINO
        ledcWrite(pwm_ch, 0);
#endif
    }
}

void MotorController::drive(int16_t left_speed, int16_t right_speed) {
    _current_left = left_speed;
    _current_right = right_speed;

    setSingleMotor(ROVER_PWM_LEFT_CHANNEL, PIN_MOTOR_LEFT_DIR1, PIN_MOTOR_LEFT_DIR2, left_speed);
    setSingleMotor(ROVER_PWM_RIGHT_CHANNEL, PIN_MOTOR_RIGHT_DIR1, PIN_MOTOR_RIGHT_DIR2, right_speed);
}

void MotorController::forward(uint8_t speed) {
    drive(speed, speed);
}

void MotorController::backward(uint8_t speed) {
    drive(-speed, -speed);
}

void MotorController::turnLeft(uint8_t speed) {
    drive(-speed, speed);
}

void MotorController::turnRight(uint8_t speed) {
    drive(speed, -speed);
}

void MotorController::stop() {
    drive(0, 0);
}
