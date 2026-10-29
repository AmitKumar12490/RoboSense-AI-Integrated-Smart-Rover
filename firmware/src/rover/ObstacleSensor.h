#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * @class ObstacleSensor
 * @brief Interfaces with HC-SR04 ultrasonic rangefinder and IR optical proximity sensors.
 */
class ObstacleSensor {
public:
    ObstacleSensor();

    /**
     * @brief Configure sensor trigger, echo, and digital IR pins.
     */
    void begin();

    /**
     * @brief Trigger ultrasonic ping and compute distance in centimeters.
     * @return Distance in cm, or 400.0f if no echo received (timeout).
     */
    float readUltrasonicDistanceCm();

    /**
     * @brief Read left and right digital infrared obstacle detector states.
     * @return true if an obstacle is detected in close proximity (active LOW on typical modules).
     */
    bool isLeftBlocked();
    bool isRightBlocked();

    float getLastDistanceCm() const { return _last_distance_cm; }

private:
    float _last_distance_cm;
    float _history[5];
    uint8_t _history_idx;
};
