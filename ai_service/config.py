import os
from typing import Optional

# Configuration for RoboSense AI Integration
OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY", None)
OPENAI_MODEL: str = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

# Serial Bridge Configuration
ESP32_SERIAL_PORT: str = os.getenv("ESP32_SERIAL_PORT", "COM3")
ESP32_BAUD_RATE: int = int(os.getenv("ESP32_BAUD_RATE", "115200"))

# Strict Whitelist of firmware-accepted commands
WHITELISTED_COMMANDS = {
    "START_MAGLEV",
    "STOP_MAGLEV",
    "STATUS",
    "CALIBRATE",
    "ROVER_MODE",
    "SAFE_MODE",
    "EMERGENCY_STOP",
    "RESET_FAULTS",
}

# Strictly forbidden command patterns (direct hardware manipulation)
FORBIDDEN_KEYWORDS = [
    "gpio", "pin", "register", "duty_cycle", "raw_pwm", "direct_write",
    "set_pin", "digitalwrite", "analogwrite", "exec", "eval", "system"
]
