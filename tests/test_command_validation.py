import pytest
from ai_service.command_validator import CommandValidator

def test_valid_commands_pass():
    valid_list = [
        {"command": "START_MAGLEV", "mode": "MAGLEV"},
        {"command": "STOP_MAGLEV", "mode": "MAGLEV"},
        {"command": "STATUS", "mode": "ANY"},
        {"command": "CALIBRATE", "mode": "MAGLEV"},
        {"command": "ROVER_MODE", "mode": "ROVER"},
        {"command": "SAFE_MODE", "mode": "SAFE"},
        {"command": "EMERGENCY_STOP", "mode": "SAFE"},
        {"command": "RESET_FAULTS", "mode": "SAFE"},
    ]
    for cmd in valid_list:
        is_valid, msg, sanitized = CommandValidator.validate_command(cmd)
        assert is_valid is True, f"Failed for valid command {cmd}: {msg}"
        assert sanitized["command"] == cmd["command"]


def test_unwhitelisted_commands_rejected():
    invalid_list = [
        {"command": "REBOOT_CPU"},
        {"command": "OVERCLOCK"},
        {"command": "FLY_AWAY"},
        {"command": "DELETE_FLASH"},
    ]
    for cmd in invalid_list:
        is_valid, msg, _ = CommandValidator.validate_command(cmd)
        assert is_valid is False
        assert "not in the firmware whitelist" in msg


def test_hardware_injection_rejected():
    malicious_payloads = [
        {"command": "START_MAGLEV", "gpio": 23, "val": 1},
        {"command": "STATUS", "raw_pwm": 1023},
        {"command": "START_MAGLEV", "duty_cycle": 100},
        {"command": "SAFE_MODE", "register_write": "0x3FF44000"},
        {"command": "digitalWrite", "pin": 23},
    ]
    for payload in malicious_payloads:
        is_valid, msg, _ = CommandValidator.validate_command(payload)
        assert is_valid is False
        assert "Security Violation" in msg or "not in the firmware whitelist" in msg


def test_advisory_tuning_is_never_auto_executable():
    advice = {
        "suggested_direction": "Reduce Kp by 10%",
        "rationale": "High-frequency limit cycle oscillation observed."
    }
    sanitized_advice = CommandValidator.sanitize_tuning_advice(advice)
    assert sanitized_advice["is_advisory"] is True
    assert sanitized_advice["automated_execution_blocked"] is True
    assert "automated_execution_blocked" in sanitized_advice


if __name__ == "__main__":
    test_valid_commands_pass()
    test_unwhitelisted_commands_rejected()
    test_hardware_injection_rejected()
    test_advisory_tuning_is_never_auto_executable()
    print("All AI command validation security tests passed successfully!")
