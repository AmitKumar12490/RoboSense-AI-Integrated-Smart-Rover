import json
import logging
from typing import Dict, Any, Tuple
from .config import WHITELISTED_COMMANDS, FORBIDDEN_KEYWORDS

logger = logging.getLogger("RoboSense.SafetyValidator")

class SecurityValidationError(ValueError):
    """Raised when an AI-generated command violates embedded safety policies."""
    pass

class CommandValidator:
    """
    Deterministic safety gateway between AI models and firmware execution.
    Guarantees that untrusted LLM outputs cannot directly access hardware registers,
    GPIO numbers, or unvalidated PWM duty cycles.
    """

    @classmethod
    def validate_command(cls, payload: Dict[str, Any]) -> Tuple[bool, str, Dict[str, Any]]:
        """
        Validates structured command payload from AI intent parser.
        Returns: (is_valid, error_or_success_message, sanitized_payload)
        """
        if not isinstance(payload, dict):
            return False, "Payload must be a valid JSON dictionary", {}

        command = payload.get("command", "").strip().upper()
        if not command:
            return False, "Missing 'command' key in payload", {}

        # 1. Check for injected forbidden hardware control keywords
        serialized_str = json.dumps(payload).lower()
        for forbidden in FORBIDDEN_KEYWORDS:
            if forbidden in serialized_str:
                logger.critical(f"Security Alert: Blocked attempt to inject forbidden keyword '{forbidden}'")
                return False, f"Security Violation: '{forbidden}' is forbidden in command layer", {}

        # 2. Strict whitelist verification
        if command not in WHITELISTED_COMMANDS:
            logger.warning(f"Rejected unknown/unwhitelisted command: {command}")
            return False, f"Command '{command}' is not in the firmware whitelist", {}

        # 3. Sanitize and prepare clean payload
        sanitized = {
            "command": command,
            "mode": payload.get("mode", ""),
            "origin": "AI_GATEWAY_VALIDATED"
        }

        return True, f"Command '{command}' validated successfully", sanitized

    @classmethod
    def sanitize_tuning_advice(cls, advice_payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        Ensures AI tuning recommendations are marked strictly advisory
        and cannot be executed automatically by any firmware routine.
        """
        return {
            "is_advisory": True,
            "automated_execution_blocked": True,
            "tuning_direction": advice_payload.get("suggested_direction", "Maintain current parameters"),
            "rationale": advice_payload.get("rationale", "No instability detected"),
            "warning": "CRITICAL: AI tuning suggestions must be validated by human operator on test bench."
        }
