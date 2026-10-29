#include "SensorCalibration.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    // Standalone mock for unit testing on host environments
    static uint16_t s_mock_adc_val = 2048;
    static uint16_t analogRead(uint8_t pin) { (void)pin; return s_mock_adc_val; }
    static void delay(uint32_t ms) { (void)ms; }
#endif

SensorCalibration::SensorCalibration()
    : _adc_pin(PIN_MAGLEV_SENSOR_ADC),
      _baseline_offset(2048.0f), // Default mid-scale for ratiometric Hall sensor
      _filtered_position(0.0f),
      _last_raw(2048),
      _filter_alpha(SENSOR_EMA_ALPHA),
      _is_calibrated(false) {}

void SensorCalibration::begin(uint8_t adc_pin) {
    _adc_pin = adc_pin;
#ifdef ARDUINO
    pinMode(_adc_pin, INPUT);
    // Configure ESP32 ADC attenuation for 0 - 3.1V dynamic range
    analogSetAttenuation(ADC_11db);
#endif
    _is_calibrated = false;
    _filtered_position = 0.0f;
}

uint16_t SensorCalibration::readRawHardware() {
    _last_raw = analogRead(_adc_pin);
    return _last_raw;
}

bool SensorCalibration::isSensorValid(uint16_t raw_adc) const {
    return (raw_adc >= SENSOR_RAW_MIN_VALID && raw_adc <= SENSOR_RAW_MAX_VALID);
}

bool SensorCalibration::calibrateBaseline(uint16_t sample_count) {
    if (sample_count == 0) return false;

    uint32_t accumulator = 0;
    uint16_t valid_count = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        uint16_t sample = readRawHardware();
        if (isSensorValid(sample)) {
            accumulator += sample;
            valid_count++;
        }
        delay(2);
    }

    if (valid_count < (sample_count * 3 / 4)) {
        // More than 25% invalid samples detected: sensor is erratic or unplugged
        _is_calibrated = false;
        return false;
    }

    _baseline_offset = static_cast<float>(accumulator) / static_cast<float>(valid_count);
    _filtered_position = 0.0f;
    _is_calibrated = true;
    return true;
}

float SensorCalibration::update(uint16_t raw_adc) {
    _last_raw = raw_adc;

    if (!isSensorValid(raw_adc)) {
        // Return current filtered position without corrupting state on transient glitch
        return _filtered_position;
    }

    // Relative position calculation: deviation from calibrated tare baseline
    float relative_pos = static_cast<float>(raw_adc) - _baseline_offset;

    // Apply Exponential Moving Average (EMA) low-pass smoothing
    // y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
    _filtered_position = (_filter_alpha * relative_pos) + 
                         ((1.0f - _filter_alpha) * _filtered_position);

    return _filtered_position;
}
