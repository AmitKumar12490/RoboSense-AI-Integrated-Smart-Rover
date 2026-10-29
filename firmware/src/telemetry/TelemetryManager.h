#pragma once

#include <stdint.h>
#include "SystemTypes.h"

/**
 * @class TelemetryManager
 * @brief Streams formatted serial debug lines and structured JSON telemetry.
 */
class TelemetryManager {
public:
    TelemetryManager();

    void begin(uint32_t baud_rate = 115200);

    /**
     * @brief Print human-readable serial monitor log line for MagLev loop.
     */
    void printMagLevDebug(const MagLevTelemetry& data);

    /**
     * @brief Emit structured JSON payload for external AI and dashboard consumers.
     */
    void emitMagLevJson(const MagLevTelemetry& data);

    /**
     * @brief Emit structured JSON payload for Rover subsystem.
     */
    void emitRoverJson(const RoverTelemetry& data);

    /**
     * @brief Print system status notification.
     */
    void printStatusMessage(const char* prefix, const char* message);
};
