/**
 * @file main.cpp
 * @brief RoboSense - Dual-Subsystem Firmware (Autonomous Rover + Electromagnetic Levitation)
 *
 * Implements deterministic high-frequency PID control for magnetic levitation,
 * autonomous obstacle avoidance navigation, hardware safety supervisor, and
 * a validated serial command parser interfaced with OpenAI API intelligence.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "SystemTypes.h"
#include "maglev/MagLevController.h"
#include "rover/RoverSubsystem.h"
#include "safety/SafetySupervisor.h"
#include "telemetry/TelemetryManager.h"

// Subsystem instances
static SafetySupervisor  g_supervisor;
static MagLevController  g_maglev;
static RoverSubsystem   g_rover;
static TelemetryManager  g_telemetry;

// Global Operating Mode (Default on startup is strictly SAFE_MODE)
static SystemMode g_current_mode = SystemMode::SAFE_MODE;

// Timing tickers for deterministic non-blocking scheduling
static uint32_t g_last_maglev_micros = 0;
static uint32_t g_last_rover_millis = 0;
static uint32_t g_last_telemetry_millis = 0;

// Serial input buffer
static char g_serial_rx_buffer[128];
static uint8_t g_rx_index = 0;

// Forward declarations
void processIncomingSerial();
void handleCommand(const char* cmd_line);
void setSystemMode(SystemMode new_mode);

void setup() {
    // 1. Initialize Serial communication with PC / AI Gateway
    g_telemetry.begin(115200);
    delay(200); // Allow UART voltage rails to stabilize

    g_telemetry.printStatusMessage("SYS", "==================================================");
    g_telemetry.printStatusMessage("SYS", "RoboSense: AI-Integrated Smart Rover & MagLev Init");
    g_telemetry.printStatusMessage("SYS", "==================================================");

    // 2. Initialize Safety Supervisor
    g_supervisor.begin();

    // 3. Initialize Actuator and Sensor Subsystems
    g_maglev.begin(&g_supervisor);
    g_rover.begin();

    // 4. Force default state: SAFE_MODE (Motors and Electromagnet strictly unpowered)
    setSystemMode(SystemMode::SAFE_MODE);

    g_last_maglev_micros = micros();
    g_last_rover_millis = millis();
    g_last_telemetry_millis = millis();

    g_telemetry.printStatusMessage("SYS", "System Ready. Mode: SAFE_MODE. Actuators Isolated.");
}

void loop() {
    uint32_t current_micros = micros();
    uint32_t current_millis = millis();

    // ------------------------------------------------------------------------
    // 1. HIGH-FREQUENCY DETERMINISTIC MAGLEV CONTROL LOOP (500 Hz / 2000 us)
    // ------------------------------------------------------------------------
    uint32_t elapsed_maglev_us = current_micros - g_last_maglev_micros;
    if (elapsed_maglev_us >= MAGLEV_CONTROL_INTERVAL_US) {
        g_last_maglev_micros = current_micros;

        if (g_current_mode == SystemMode::MAGLEV_MODE) {
            g_maglev.update(elapsed_maglev_us);
        } else {
            // Strictly enforce zero PWM to coil outside MAGLEV_MODE
            g_maglev.stop();
        }
    }

    // ------------------------------------------------------------------------
    // 2. ROVER NAVIGATION & OBSTACLE AVOIDANCE EVALUATION LOOP (20 Hz / 50 ms)
    // ------------------------------------------------------------------------
    uint32_t elapsed_rover_ms = current_millis - g_last_rover_millis;
    if (elapsed_rover_ms >= ROVER_CONTROL_INTERVAL_MS) {
        g_last_rover_millis = current_millis;

        if (g_current_mode == SystemMode::ROVER_MODE) {
            g_rover.update(elapsed_rover_ms);
        } else {
            // Strictly enforce zero power to traction motors outside ROVER_MODE
            g_rover.stop();
        }
    }

    // ------------------------------------------------------------------------
    // 3. TELEMETRY STREAMING & DEBUG BROADCAST (20 Hz / 50 ms)
    // ------------------------------------------------------------------------
    if (current_millis - g_last_telemetry_millis >= TELEMETRY_INTERVAL_MS) {
        g_last_telemetry_millis = current_millis;

        if (g_current_mode == SystemMode::MAGLEV_MODE) {
            MagLevTelemetry maglev_data = g_maglev.getTelemetry(g_current_mode);
            g_telemetry.emitMagLevJson(maglev_data);
        } else if (g_current_mode == SystemMode::ROVER_MODE) {
            RoverTelemetry rover_data = g_rover.getTelemetry(g_current_mode);
            g_telemetry.emitRoverJson(rover_data);
        } else {
            // In SAFE_MODE, emit heartbeat
            MagLevTelemetry safe_data = g_maglev.getTelemetry(g_current_mode);
            g_telemetry.emitMagLevJson(safe_data);
        }
    }

    // ------------------------------------------------------------------------
    // 4. PARSE INCOMING SERIAL COMMANDS (FROM AI GATEWAY / USER TERMINAL)
    // ------------------------------------------------------------------------
    processIncomingSerial();
}

/**
 * @brief Enforces hardware interlock when transitioning between system modes.
 */
void setSystemMode(SystemMode new_mode) {
    if (new_mode == g_current_mode) return;

    // Safety interlock check
    if (!g_supervisor.verifyInterlock(new_mode)) {
        g_telemetry.printStatusMessage("SAFETY", "Hardware interlock transition violation rejected!");
        return;
    }

    // Stop all active actuators before transitioning
    g_maglev.stop();
    g_rover.stop();

    g_current_mode = new_mode;

    char msg[64];
    snprintf(msg, sizeof(msg), "Mode changed to %s", systemModeToString(g_current_mode));
    g_telemetry.printStatusMessage("SYS", msg);
}

/**
 * @brief Reads incoming characters from Serial port non-blockingly.
 */
void processIncomingSerial() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            g_serial_rx_buffer[g_rx_index] = '\0';
            if (g_rx_index > 0) {
                handleCommand(g_serial_rx_buffer);
            }
            g_rx_index = 0;
        } else {
            if (g_rx_index < sizeof(g_serial_rx_buffer) - 1) {
                g_serial_rx_buffer[g_rx_index++] = c;
            }
        }
    }
}

/**
 * @brief Validates and executes strictly whitelisted commands.
 * Raw pin writes, arbitrary memory access, and unchecked duty cycles are FORBIDDEN.
 */
void handleCommand(const char* cmd_line) {
    // 1. JSON-formatted structured command from AI Gateway
    if (cmd_line[0] == '{') {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, cmd_line);
        if (!err) {
            const char* command = doc["command"] | "";
            if (strcmp(command, "START_MAGLEV") == 0) {
                if (g_current_mode != SystemMode::MAGLEV_MODE) {
                    setSystemMode(SystemMode::MAGLEV_MODE);
                }
                g_maglev.start();
                g_telemetry.printStatusMessage("CMD", "MagLev closed-loop started");
            } else if (strcmp(command, "STOP_MAGLEV") == 0) {
                g_maglev.stop();
                g_telemetry.printStatusMessage("CMD", "MagLev closed-loop stopped");
            } else if (strcmp(command, "CALIBRATE") == 0) {
                g_telemetry.printStatusMessage("CMD", "Calibrating MagLev baseline sensor...");
                g_maglev.calibrate();
            } else if (strcmp(command, "ROVER_MODE") == 0) {
                setSystemMode(SystemMode::ROVER_MODE);
                g_rover.enableAutonomousMode();
            } else if (strcmp(command, "SAFE_MODE") == 0) {
                setSystemMode(SystemMode::SAFE_MODE);
            } else if (strcmp(command, "STATUS") == 0) {
                MagLevTelemetry t = g_maglev.getTelemetry(g_current_mode);
                g_telemetry.printMagLevDebug(t);
            } else {
                g_telemetry.printStatusMessage("CMD_ERR", "Unknown structured command");
            }
            return;
        }
    }

    // 2. Standard ASCII Terminal commands
    if (strncmp(cmd_line, "MODE ", 5) == 0) {
        const char* mode_arg = cmd_line + 5;
        if (strcmp(mode_arg, "SAFE") == 0) {
            setSystemMode(SystemMode::SAFE_MODE);
        } else if (strcmp(mode_arg, "ROVER") == 0) {
            setSystemMode(SystemMode::ROVER_MODE);
        } else if (strcmp(mode_arg, "MAGLEV") == 0) {
            setSystemMode(SystemMode::MAGLEV_MODE);
        } else {
            g_telemetry.printStatusMessage("CMD_ERR", "Invalid MODE. Choose: SAFE, ROVER, MAGLEV");
        }
    } else if (strcmp(cmd_line, "START_MAGLEV") == 0) {
        if (g_current_mode != SystemMode::MAGLEV_MODE) {
            setSystemMode(SystemMode::MAGLEV_MODE);
        }
        g_maglev.start();
        g_telemetry.printStatusMessage("CMD", "MagLev loop started");
    } else if (strcmp(cmd_line, "STOP_MAGLEV") == 0) {
        g_maglev.stop();
        g_telemetry.printStatusMessage("CMD", "MagLev loop stopped");
    } else if (strcmp(cmd_line, "CALIBRATE") == 0) {
        g_telemetry.printStatusMessage("CMD", "Running baseline tare calibration...");
        g_maglev.calibrate();
    } else if (strcmp(cmd_line, "STATUS") == 0) {
        MagLevTelemetry t = g_maglev.getTelemetry(g_current_mode);
        g_telemetry.printMagLevDebug(t);
    } else if (strcmp(cmd_line, "RESET_FAULTS") == 0) {
        g_supervisor.resetFaults();
        g_telemetry.printStatusMessage("CMD", "Safety supervisor faults cleared");
    } else if (strcmp(cmd_line, "EMERGENCY_STOP") == 0) {
        g_maglev.stop();
        g_rover.stop();
        setSystemMode(SystemMode::SAFE_MODE);
        g_supervisor.triggerEmergencyStop(FaultReason::NONE);
        g_telemetry.printStatusMessage("SAFETY", "EMERGENCY STOP ENGAGED. All actuators disabled.");
    } else if (strncmp(cmd_line, "SET_PID ", 8) == 0) {
        float kp, ki, kd;
        if (sscanf(cmd_line + 8, "%f %f %f", &kp, &ki, &kd) == 3) {
            if (g_maglev.setTunings(kp, ki, kd)) {
                g_telemetry.printStatusMessage("CMD", "PID tunings updated successfully");
            } else {
                g_telemetry.printStatusMessage("CMD_ERR", "PID tunings out of safe bounds");
            }
        } else {
            g_telemetry.printStatusMessage("CMD_ERR", "Usage: SET_PID <kp> <ki> <kd>");
        }
    } else if (strncmp(cmd_line, "SET_TARGET ", 11) == 0) {
        float target;
        if (sscanf(cmd_line + 11, "%f", &target) == 1) {
            g_maglev.setTarget(target);
            g_telemetry.printStatusMessage("CMD", "Target setpoint updated");
        } else {
            g_telemetry.printStatusMessage("CMD_ERR", "Usage: SET_TARGET <value>");
        }
    } else if (strncmp(cmd_line, "ROVER_CMD ", 10) == 0) {
        if (g_current_mode != SystemMode::ROVER_MODE) {
            g_telemetry.printStatusMessage("CMD_ERR", "Must be in ROVER_MODE to drive motors");
            return;
        }
        const char* rover_arg = cmd_line + 10;
        if (strcmp(rover_arg, "AUTO") == 0) {
            g_rover.enableAutonomousMode();
            g_telemetry.printStatusMessage("ROVER", "Autonomous obstacle avoidance active");
        } else if (strcmp(rover_arg, "FWD") == 0) {
            g_rover.setManualDrive(180, 180);
        } else if (strcmp(rover_arg, "REV") == 0) {
            g_rover.setManualDrive(-160, -160);
        } else if (strcmp(rover_arg, "LEFT") == 0) {
            g_rover.setManualDrive(-160, 160);
        } else if (strcmp(rover_arg, "RIGHT") == 0) {
            g_rover.setManualDrive(160, -160);
        } else if (strcmp(rover_arg, "STOP") == 0) {
            g_rover.stop();
        } else {
            g_telemetry.printStatusMessage("CMD_ERR", "Invalid ROVER_CMD: AUTO, FWD, REV, LEFT, RIGHT, STOP");
        }
    } else {
        g_telemetry.printStatusMessage("CMD_ERR", "Unrecognized command. Type STATUS for current state.");
    }
}
