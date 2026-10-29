import os
import json
import logging
from typing import Dict, Any, Optional
from .config import OPENAI_API_KEY, OPENAI_MODEL
from .command_validator import CommandValidator

logger = logging.getLogger("RoboSense.AIAgent")

SYSTEM_INTENT_PROMPT = """
You are the high-level natural language command parser for RoboSense - an AI-Integrated Smart Rover and Magnetic Levitation system.
Your job is to translate the user's natural language instruction into a single validated command dictionary.
ALLOWED COMMANDS:
- START_MAGLEV (mode: MAGLEV)
- STOP_MAGLEV (mode: MAGLEV)
- STATUS (mode: ANY)
- CALIBRATE (mode: MAGLEV)
- ROVER_MODE (mode: ROVER)
- SAFE_MODE (mode: SAFE)
- EMERGENCY_STOP (mode: SAFE)
- RESET_FAULTS (mode: SAFE)

CRITICAL SECURITY RULES:
- You must NEVER generate GPIO numbers, pin assignments, raw PWM values, or direct register writes.
- Output ONLY valid JSON with keys: "command", "mode", "explanation".
"""

SYSTEM_DIAGNOSTIC_PROMPT = """
You are an expert control systems and embedded robotics engineer analyzing telemetry from an electromagnetic levitation (MagLev) apparatus.
Analyze the provided sensor telemetry payload:
- Target position, measured position, error, PID terms, PWM duty cycle, and system status.
Diagnose:
1. Operational status summary.
2. Stability evaluation (e.g. steady-state error, ringing, near saturation, object falling).
3. Probable physical causes.
4. Suggested manual PID tuning direction (Advisory only!).

CRITICAL SAFETY RULE:
- Clearly state that AI recommendations NEVER automatically modify PID registers or duty cycles on the microcontroller.
- Return structured JSON with keys: "status_summary", "stability_assessment", "possible_causes", "tuning_advice", "warning".
"""

class AIAgent:
    """
    AI Integration agent interfacing OpenAI API with the embedded RoboSense platform.
    Features robust offline fallback heuristics when no API key is provisioned.
    """

    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        self.api_key = api_key or OPENAI_API_KEY
        self.model = model or OPENAI_MODEL
        self._openai_client = None

        if self.api_key and self.api_key != "your_openai_api_key_here":
            try:
                from openai import OpenAI
                self._openai_client = OpenAI(api_key=self.api_key)
                logger.info(f"OpenAI client initialized with model {self.model}")
            except Exception as e:
                logger.warning(f"Could not initialize OpenAI client ({e}). Operating in deterministic rule mode.")

    def parse_user_command(self, user_prompt: str) -> Dict[str, Any]:
        """
        Translates human natural language to structured, validated firmware commands.
        """
        raw_intent = None

        # 1. Attempt OpenAI API completion if client is active
        if self._openai_client:
            try:
                response = self._openai_client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": SYSTEM_INTENT_PROMPT},
                        {"role": "user", "content": user_prompt}
                    ],
                    response_format={"type": "json_object"},
                    temperature=0.0
                )
                raw_intent = json.loads(response.choices[0].message.content)
            except Exception as e:
                logger.error(f"OpenAI API error during command parsing: {e}. Falling back to rule parser.")

        # 2. Offline fallback rule parser if OpenAI unavailable or errored
        if not raw_intent:
            raw_intent = self._offline_rule_intent_parser(user_prompt)

        # 3. Pass through strict hardware security validator
        is_valid, message, sanitized = CommandValidator.validate_command(raw_intent)
        sanitized["validation_status"] = "PASSED" if is_valid else "REJECTED"
        sanitized["validation_message"] = message
        sanitized["original_prompt"] = user_prompt
        sanitized["explanation"] = raw_intent.get("explanation", message)

        return sanitized

    def diagnose_telemetry(self, telemetry: Dict[str, Any]) -> Dict[str, Any]:
        """
        Analyzes real-time MagLev telemetry and generates control diagnostics and tuning guidance.
        """
        # 1. Attempt OpenAI API analysis if available
        if self._openai_client:
            try:
                response = self._openai_client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": SYSTEM_DIAGNOSTIC_PROMPT},
                        {"role": "user", "content": json.dumps(telemetry)}
                    ],
                    response_format={"type": "json_object"},
                    temperature=0.2
                )
                diag = json.loads(response.choices[0].message.content)
                return CommandValidator.sanitize_tuning_advice(diag)
            except Exception as e:
                logger.error(f"OpenAI API error during telemetry diagnosis: {e}. Falling back to rule-based diagnostics.")

        # 2. Rule-based offline control-systems diagnostic engine
        return self._offline_telemetry_diagnostic(telemetry)

    def _offline_rule_intent_parser(self, prompt: str) -> Dict[str, Any]:
        """Deterministic NLP intent extraction for testing and offline operation."""
        p = prompt.strip().lower()

        if "emergency" in p or "abort" in p or "halt" in p or ("stop" in p and ("all" in p or "power" in p or "emergency" in p)):
            return {"command": "EMERGENCY_STOP", "mode": "SAFE", "explanation": "Emergency abort requested"}
        elif "calibrat" in p or "tare" in p:
            return {"command": "CALIBRATE", "mode": "MAGLEV", "explanation": "Calibrating sensor baseline offset"}
        elif "stop" in p and ("maglev" in p or "levitat" in p):
            return {"command": "STOP_MAGLEV", "mode": "MAGLEV", "explanation": "Disabling electromagnet levitation"}
        elif ("start" in p or "activate" in p or "begin" in p) and ("maglev" in p or "levitat" in p):
            return {"command": "START_MAGLEV", "mode": "MAGLEV", "explanation": "Activating electromagnetic levitation"}
        elif "rover" in p or "drive" in p or "navigate" in p or "car" in p:
            return {"command": "ROVER_MODE", "mode": "ROVER", "explanation": "Switching to rover mobile navigation mode"}
        elif "safe" in p or "idle" in p or "standby" in p:
            return {"command": "SAFE_MODE", "mode": "SAFE", "explanation": "Isolating all actuators into safe mode"}
        elif "reset" in p and "fault" in p:
            return {"command": "RESET_FAULTS", "mode": "SAFE", "explanation": "Clearing tripped safety supervisor faults"}
        elif "status" in p or "telemetry" in p or "health" in p:
            return {"command": "STATUS", "mode": "ANY", "explanation": "Requesting telemetry and status snapshot"}
        else:
            return {"command": "STATUS", "mode": "ANY", "explanation": "Unrecognized request, querying status"}

    def _offline_telemetry_diagnostic(self, t: Dict[str, Any]) -> Dict[str, Any]:
        """Physics and control-theory based diagnostic analysis."""
        error = float(t.get("error", 0.0))
        pwm = int(t.get("pwm", 0))
        state = t.get("state", "UNKNOWN")
        status = t.get("system_status", "OK")

        stability = "STABLE"
        tuning_dir = "Maintain current PID constants."
        possible_causes = []

        if status != "OK":
            stability = "FAULT"
            tuning_dir = "Resolve hardware fault condition before adjusting PID gains."
            possible_causes.append(f"Safety supervisor tripped: {status}")
        elif pwm >= 850:
            stability = "SATURATION"
            tuning_dir = "Electromagnet is operating at saturation limit. Increase external 12V supply voltage or decrease target levitation gap."
            possible_causes.append("Payload weight exceeds magnetic lift force or setpoint is too far from coil.")
        elif abs(error) > 200.0:
            stability = "UNSTABLE_DIVERGING"
            tuning_dir = "System is diverging. Consider increasing proportional gain Kp for faster restorative force, or increasing derivative gain Kd to damp oscillations."
            possible_causes.append("External mechanical vibration, air currents, or insufficient derivative damping.")
        elif abs(error) > 30.0:
            stability = "MARGINAL_OSCILLATION"
            tuning_dir = "Persistent steady-state offset or mild oscillation detected. Slightly increase integral gain Ki to eliminate steady-state error, and fine-tune Kd."
            possible_causes.append("Coil temperature rise shifting winding resistance, or slight permanent magnet misalignment.")
        else:
            stability = "BALANCED_EQUILIBRIUM"
            tuning_dir = "PID controller is maintaining equilibrium within ±30 ADC units of target setpoint."
            possible_causes.append("Optimal equilibrium between gravitational force mg and magnetic attraction F_mag.")

        advice_dict = {
            "suggested_direction": tuning_dir,
            "rationale": f"Telemetry state={state}, error={error:.1f}, PWM={pwm}, stability={stability}. " + "; ".join(possible_causes)
        }

        return CommandValidator.sanitize_tuning_advice(advice_dict)
