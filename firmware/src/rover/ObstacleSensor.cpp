#include "ObstacleSensor.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    static void pinMode(uint8_t pin, uint8_t mode) { (void)pin; (void)mode; }
    static void digitalWrite(uint8_t pin, uint8_t val) { (void)pin; (void)val; }
    static int digitalRead(uint8_t pin) { (void)pin; return 1; }
    static uint32_t pulseIn(uint8_t pin, uint8_t val, uint32_t timeout) { (void)pin; (void)val; (void)timeout; return 1200; }
    static void delayMicroseconds(uint32_t us) { (void)us; }
    #define INPUT 0
    #define OUTPUT 1
    #define HIGH 1
    #define LOW 0
#endif

ObstacleSensor::ObstacleSensor()
    : _last_distance_cm(100.0f),
      _history_idx(0) {
    for (int i = 0; i < 5; ++i) {
        _history[i] = 100.0f;
    }
}

void ObstacleSensor::begin() {
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    pinMode(PIN_IR_OBSTACLE_LEFT, INPUT);
    pinMode(PIN_IR_OBSTACLE_RIGHT, INPUT);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
}

float ObstacleSensor::readUltrasonicDistanceCm() {
#ifdef ARDUINO
    // Generate clean 10us trigger pulse
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

    // Timeout after 25,000 us (approx 4.3 meters max range)
    uint32_t duration_us = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 25000);
    if (duration_us == 0) {
        _last_distance_cm = 400.0f; // No echo received
    } else {
        // Speed of sound = 343 m/s = 0.0343 cm/us -> round trip divide by 2 (~ 58.2 us/cm)
        float distance = static_cast<float>(duration_us) / 58.2f;
        if (distance > 400.0f) distance = 400.0f;
        if (distance < 2.0f) distance = 2.0f;
        _last_distance_cm = distance;
    }
#else
    _last_distance_cm = 50.0f;
#endif

    // Update moving median filter buffer
    _history[_history_idx] = _last_distance_cm;
    _history_idx = (_history_idx + 1) % 5;

    // Compute simple median of 5 samples
    float sorted[5];
    for (int i = 0; i < 5; ++i) sorted[i] = _history[i];
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 5; ++j) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    return sorted[2]; // Return median sample
}

bool ObstacleSensor::isLeftBlocked() {
    // IR optical sensors are typically active-LOW when infrared reflection is detected
    return digitalRead(PIN_IR_OBSTACLE_LEFT) == LOW;
}

bool ObstacleSensor::isRightBlocked() {
    return digitalRead(PIN_IR_OBSTACLE_RIGHT) == LOW;
}
