import pytest

class DiscreteSafetySupervisor:
    """Exact python mirror of firmware/src/safety/SafetySupervisor.cpp"""
    def __init__(self):
        self.current_fault = "NONE"
        self.high_pwm_accumulator_ms = 0
        self.out_of_bounds_count = 0

    def reset_faults(self):
        self.current_fault = "NONE"
        self.high_pwm_accumulator_ms = 0
        self.out_of_bounds_count = 0

    def trigger_emergency_stop(self, reason: str):
        self.current_fault = reason

    def check_maglev_safety(self, raw_sensor: int, error: float, pwm_duty: int, dt_ms: int) -> bool:
        if raw_sensor < 60:
            self.trigger_emergency_stop("SENSOR_DISCONNECTED")
            return False
        if raw_sensor > 4035:
            self.trigger_emergency_stop("SENSOR_SATURATED")
            return False

        if abs(error) > 620.0:
            self.out_of_bounds_count += 1
            if self.out_of_bounds_count > 15:
                self.trigger_emergency_stop("OBJECT_DROPPED")
                return False
        else:
            self.out_of_bounds_count = 0

        if pwm_duty >= 800:
            self.high_pwm_accumulator_ms += dt_ms
            if self.high_pwm_accumulator_ms >= 3500:
                self.trigger_emergency_stop("THERMAL_OVERHEAT_TIMEOUT")
                return False
        else:
            if self.high_pwm_accumulator_ms >= dt_ms:
                self.high_pwm_accumulator_ms -= dt_ms
            else:
                self.high_pwm_accumulator_ms = 0

        return True

    def verify_mode_interlock(self, current_mode: str, requested_mode: str,
                              rover_pwm: int, maglev_pwm: int) -> bool:
        # Enforce strict mutual exclusion: motors and coil can never operate concurrently
        if requested_mode == "ROVER_MODE":
            if maglev_pwm != 0:
                return False
            return True
        elif requested_mode == "MAGLEV_MODE":
            if rover_pwm != 0:
                return False
            return True
        elif requested_mode == "SAFE_MODE":
            return True
        return False


def test_sensor_disconnection_trips_fault():
    sup = DiscreteSafetySupervisor()
    safe = sup.check_maglev_safety(raw_sensor=25, error=0.0, pwm_duty=100, dt_ms=2)
    assert safe is False
    assert sup.current_fault == "SENSOR_DISCONNECTED"


def test_sensor_saturation_trips_fault():
    sup = DiscreteSafetySupervisor()
    safe = sup.check_maglev_safety(raw_sensor=4090, error=0.0, pwm_duty=100, dt_ms=2)
    assert safe is False
    assert sup.current_fault == "SENSOR_SATURATED"


def test_thermal_timeout_trips_on_sustained_high_duty():
    sup = DiscreteSafetySupervisor()
    # Run high PWM for 3.4 seconds (3400ms) - should stay safe
    for _ in range(1700):
        safe = sup.check_maglev_safety(raw_sensor=2048, error=0.0, pwm_duty=850, dt_ms=2)
        assert safe is True

    # Run for another 200 ms (total 3600ms >= 3500ms) - must trip thermal fault
    tripped = False
    for _ in range(100):
        safe = sup.check_maglev_safety(raw_sensor=2048, error=0.0, pwm_duty=850, dt_ms=2)
        if not safe:
            tripped = True
            break

    assert tripped is True
    assert sup.current_fault == "THERMAL_OVERHEAT_TIMEOUT"


def test_hardware_mutual_exclusion_interlock():
    sup = DiscreteSafetySupervisor()
    # Transition to ROVER_MODE when MagLev coil has non-zero PWM -> FORBIDDEN
    assert sup.verify_mode_interlock(current_mode="SAFE_MODE", requested_mode="ROVER_MODE",
                                     rover_pwm=0, maglev_pwm=200) is False

    # Transition to MAGLEV_MODE when Rover motor has non-zero PWM -> FORBIDDEN
    assert sup.verify_mode_interlock(current_mode="SAFE_MODE", requested_mode="MAGLEV_MODE",
                                     rover_pwm=180, maglev_pwm=0) is False

    # Clean transition when other actuator is completely zeroed -> ALLOWED
    assert sup.verify_mode_interlock(current_mode="SAFE_MODE", requested_mode="MAGLEV_MODE",
                                     rover_pwm=0, maglev_pwm=0) is True


if __name__ == "__main__":
    test_sensor_disconnection_trips_fault()
    test_sensor_saturation_trips_fault()
    test_thermal_timeout_trips_on_sustained_high_duty()
    test_hardware_mutual_exclusion_interlock()
    print("All safety interlock tests passed successfully!")
