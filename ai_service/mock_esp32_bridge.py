import time
import math
import random
from typing import Dict, Any, Callable, Optional

class SimulatedESP32:
    """
    Virtual ESP32 firmware emulator executing the exact same PID loop dynamics,
    sensor noise, safety supervisor limits, and command processing.
    """

    def __init__(self):
        self.mode = "SAFE_MODE"
        self.maglev_state = "IDLE"
        self.target_position = 512.0
        self.baseline = 2048.0
        self.kp = 1.85
        self.ki = 0.08
        self.kd = 0.42

        # Physics simulation state
        self.z = 500.0  # Simulated levitated magnet position
        self.v = 0.0    # Velocity
        self.integral = 0.0
        self.last_error = 0.0
        self.pwm = 0
        self.fault = "OK"

    def send_command(self, cmd_dict: Dict[str, Any]) -> str:
        cmd = cmd_dict.get("command", "")
        if cmd == "START_MAGLEV":
            self.mode = "MAGLEV_MODE"
            self.maglev_state = "LEVITATING"
            self.z = self.target_position - 10.0
            self.v = 0.0
            self.integral = 0.0
            self.fault = "OK"
            return "[SYS] Mode changed to MAGLEV_MODE\n[CMD] MagLev loop started"
        elif cmd == "STOP_MAGLEV":
            self.maglev_state = "IDLE"
            self.pwm = 0
            return "[CMD] MagLev loop stopped"
        elif cmd == "CALIBRATE":
            self.baseline = 2048.0 + random.uniform(-10.0, 10.0)
            return f"[CMD] Sensor calibrated. Baseline offset = {self.baseline:.1f}"
        elif cmd == "ROVER_MODE":
            self.mode = "ROVER_MODE"
            self.maglev_state = "IDLE"
            self.pwm = 0
            return "[SYS] Mode changed to ROVER_MODE\n[ROVER] Autonomous obstacle avoidance active"
        elif cmd == "SAFE_MODE":
            self.mode = "SAFE_MODE"
            self.maglev_state = "IDLE"
            self.pwm = 0
            return "[SYS] Mode changed to SAFE_MODE. All actuators isolated."
        elif cmd == "EMERGENCY_STOP":
            self.mode = "SAFE_MODE"
            self.maglev_state = "EMERGENCY_STOP"
            self.pwm = 0
            self.fault = "EMERGENCY_STOP"
            return "[SAFETY] EMERGENCY STOP ENGAGED. All actuators disabled."
        elif cmd == "STATUS":
            return f"[STATUS] Mode={self.mode}, State={self.maglev_state}, Fault={self.fault}"
        else:
            return f"[CMD_ERR] Unrecognized command: {cmd}"

    def step(self, dt: float = 0.002) -> Dict[str, Any]:
        """Runs one physics & PID step (2ms default)."""
        if self.maglev_state == "LEVITATING":
            # Magnetic levitation physics:
            # m * d2z/dt2 = F_mag - m*g + noise
            # F_mag ~ C * (PWM / z)^2
            error = self.target_position - self.z
            self.integral = max(-400.0, min(400.0, self.integral + error * dt))
            derivative = (error - self.last_error) / dt
            self.last_error = error

            pid_calc = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            self.pwm = max(0, min(870, int(pid_calc + 450))) # 450 is nominal gravity compensation duty

            # Physics acceleration
            # gravity pushes down (-), magnetic force pulls up (+)
            f_gravity = 450.0
            f_mag = float(self.pwm) * 1.02
            accel = (f_mag - f_gravity) + random.gauss(0.0, 15.0)

            self.v += accel * dt
            self.v *= 0.98 # air drag damping
            self.z += self.v * dt

            # Safety check
            if abs(error) > 600:
                self.fault = "OBJECT_OUT_OF_BOUNDS"
                self.maglev_state = "FAULT"
                self.pwm = 0
        else:
            self.pwm = 0

        return {
            "subsystem": "maglev",
            "timestamp": int(time.time() * 1000),
            "mode": self.mode,
            "state": self.maglev_state,
            "raw_sensor": int(self.baseline + (self.z - 512.0)),
            "baseline": self.baseline,
            "position": round(self.z, 2),
            "target": round(self.target_position, 2),
            "error": round(self.target_position - self.z, 2),
            "pwm": self.pwm,
            "system_status": self.fault
        }
