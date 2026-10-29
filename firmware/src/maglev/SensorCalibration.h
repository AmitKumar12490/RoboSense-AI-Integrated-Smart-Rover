#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * @class SensorCalibration
 * @brief Handles raw ADC acquisition, baseline tare calibration,
 *        exponential smoothing, and physical validity checks.
 *
 * NOTE ON MEASUREMENT PHYSICS:
 * A Hall-effect sensor measures magnetic flux density B (Gauss/Tesla).
 * Because dipole magnetic field strength drops off inversely with the cube
 * of distance (B ~ 1/z^3), the sensor provides a monotonic relative proximity
 * signal rather than a linear metric distance.
 */
class SensorCalibration {
public:
    SensorCalibration();

    /**
     * @brief Initialize hardware pin and calibration parameters.
     */
    void begin(uint8_t adc_pin = PIN_MAGLEV_SENSOR_ADC);

    /**
     * @brief Run baseline tare calibration (averages quiescent readings).
     * @return true if calibration succeeded within valid bounds, false if invalid.
     */
    bool calibrateBaseline(uint16_t sample_count = SENSOR_CALIBRATION_SAMPLES);

    /**
     * @brief Process a raw ADC reading: applies validity check, baseline subtraction,
     *        and exponential moving average filter.
     * @param raw_adc Raw 12-bit ADC value (0 - 4095)
     * @return Filtered relative position signal
     */
    float update(uint16_t raw_adc);

    /**
     * @brief Read raw analog value directly from hardware pin.
     */
    uint16_t readRawHardware();

    // Validity and status checks
    bool isSensorValid(uint16_t raw_adc) const;
    bool isCalibrated() const { return _is_calibrated; }

    // Getters
    uint16_t getLastRaw() const { return _last_raw; }
    float getBaseline() const { return _baseline_offset; }
    float getFilteredPosition() const { return _filtered_position; }
    void setBaseline(float baseline) { _baseline_offset = baseline; _is_calibrated = true; }

private:
    uint8_t  _adc_pin;
    float    _baseline_offset;
    float    _filtered_position;
    uint16_t _last_raw;
    float    _filter_alpha;
    bool     _is_calibrated;
};
