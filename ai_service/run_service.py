import sys
import time
import json
import logging
from ai_service.ai_agent import AIAgent
from ai_service.mock_esp32_bridge import SimulatedESP32

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("RoboSense.Service")

def main():
    print("=" * 65)
    print(" RoboSense - AI Integration & Telemetry Diagnostic Service ")
    print("=" * 65)

    agent = AIAgent()
    esp32 = SimulatedESP32()

    demo_queries = [
        "Please calibrate the levitation sensor before starting.",
        "Activate electromagnetic levitation mode and begin balancing.",
        "Query current telemetry and system health.",
        "Switch the rover to autonomous obstacle avoidance mode.",
        "Emergency halt! Shut down all power immediately."
    ]

    print("\n--- Phase 1: Natural Language Command Parsing & Whitelist Validation ---")
    for query in demo_queries:
        print(f"\nUser Query: '{query}'")
        validated_cmd = agent.parse_user_command(query)
        print(f"-> Parsed Intent : {validated_cmd.get('command')}")
        print(f"-> Target Mode   : {validated_cmd.get('mode')}")
        print(f"-> Security Check: {validated_cmd.get('validation_status')} ({validated_cmd.get('validation_message')})")

        # Dispatch validated command to ESP32
        esp32_response = esp32.send_command(validated_cmd)
        print(f"-> ESP32 Response: {esp32_response.strip()}")

    print("\n\n--- Phase 2: MagLev Closed-Loop Telemetry Diagnostics & Advisory ---")
    # Start levitation simulation and collect a few frames
    esp32.send_command({"command": "START_MAGLEV"})
    for i in range(10):
        t = esp32.step(dt=0.002)

    telemetry_sample = esp32.step(dt=0.002)
    print("\nReal-time Telemetry Snapshot from ESP32:")
    print(json.dumps(telemetry_sample, indent=2))

    print("\nInvoking AI Diagnostic Engine...")
    diagnostics = agent.diagnose_telemetry(telemetry_sample)
    print("\nAI Control Systems Diagnostic:")
    print(f"-> Suggested Direction : {diagnostics.get('tuning_direction')}")
    print(f"-> Physical Rationale  : {diagnostics.get('rationale')}")
    print(f"-> Safety Interlock    : {diagnostics.get('warning')}")
    print(f"-> Automated Write Blocked: {diagnostics.get('automated_execution_blocked')}")

    print("\n" + "=" * 65)
    print(" RoboSense AI Gateway Demo Finished Successfully. ")
    print("=" * 65)

if __name__ == "__main__":
    main()
