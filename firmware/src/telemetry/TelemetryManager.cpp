#include "TelemetryManager.h"
#include <stdio.h>

#ifdef ARDUINO
    #include <Arduino.h>
    #include <ArduinoJson.h>
#else
    // Host mock
#endif

TelemetryManager::TelemetryManager() {}

void TelemetryManager::begin(uint32_t baud_rate) {
#ifdef ARDUINO
    Serial.begin(baud_rate);
#else
    (void)baud_rate;
#endif
}

void TelemetryManager::printMagLevDebug(const MagLevTelemetry& data) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "[MAGLEV] Target: %6.1f | Measured: %6.1f | Error: %6.1f | PID: %6.1f (P:%5.1f I:%5.1f D:%5.1f) | PWM: %4u | State: %-10s | Status: %s",
             data.target_position,
             data.filtered_position,
             data.error,
             data.pid_output,
             data.p_term,
             data.i_term,
             data.d_term,
             data.pwm_duty,
             magLevStateToString(data.control_state),
             faultReasonToString(data.fault_code));

#ifdef ARDUINO
    Serial.println(buffer);
#else
    printf("%s\n", buffer);
#endif
}

void TelemetryManager::emitMagLevJson(const MagLevTelemetry& data) {
#ifdef ARDUINO
    JsonDocument doc;
    doc["subsystem"] = "maglev";
    doc["timestamp"] = data.timestamp_ms;
    doc["mode"] = systemModeToString(data.current_mode);
    doc["state"] = magLevStateToString(data.control_state);
    doc["raw_sensor"] = data.raw_sensor;
    doc["baseline"] = data.baseline_offset;
    doc["position"] = data.filtered_position;
    doc["target"] = data.target_position;
    doc["error"] = data.error;
    doc["p_term"] = data.p_term;
    doc["i_term"] = data.i_term;
    doc["d_term"] = data.d_term;
    doc["pid_output"] = data.pid_output;
    doc["pwm"] = data.pwm_duty;
    doc["system_status"] = faultReasonToString(data.fault_code);

    serializeJson(doc, Serial);
    Serial.println();
#endif
}

void TelemetryManager::emitRoverJson(const RoverTelemetry& data) {
#ifdef ARDUINO
    JsonDocument doc;
    doc["subsystem"] = "rover";
    doc["timestamp"] = data.timestamp_ms;
    doc["mode"] = systemModeToString(data.current_mode);
    doc["rover_status"] = data.rover_status;
    doc["dist_cm"] = data.ultrasonic_distance_cm;
    doc["ir_left"] = data.ir_left_blocked ? 1 : 0;
    doc["ir_right"] = data.ir_right_blocked ? 1 : 0;
    doc["speed_left"] = data.left_motor_speed;
    doc["speed_right"] = data.right_motor_speed;

    serializeJson(doc, Serial);
    Serial.println();
#endif
}

void TelemetryManager::printStatusMessage(const char* prefix, const char* message) {
#ifdef ARDUINO
    Serial.print("[");
    Serial.print(prefix);
    Serial.print("] ");
    Serial.println(message);
#else
    printf("[%s] %s\n", prefix, message);
#endif
}
