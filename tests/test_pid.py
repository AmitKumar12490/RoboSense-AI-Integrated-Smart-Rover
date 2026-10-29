import pytest

class DiscretePID:
    """Exact python mirror of firmware/src/maglev/PIDController.cpp"""
    def __init__(self, kp=1.85, ki=0.08, kd=0.42, min_out=0, max_out=870,
                 integral_min=-400.0, integral_max=400.0, deriv_alpha=0.25):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.deriv_alpha = deriv_alpha

        self.integral_sum = 0.0
        self.last_error = 0.0
        self.filtered_deriv = 0.0
        self.is_first_run = True

    def reset(self):
        self.integral_sum = 0.0
        self.last_error = 0.0
        self.filtered_deriv = 0.0
        self.is_first_run = True

    def compute(self, target: float, measured: float, dt: float) -> float:
        if dt <= 0.0:
            return self.min_out

        error = target - measured

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term with anti-windup clamping
        self.integral_sum += error * dt
        if self.integral_sum > self.integral_max:
            self.integral_sum = self.integral_max
        elif self.integral_sum < self.integral_min:
            self.integral_sum = self.integral_min
        i_term = self.ki * self.integral_sum

        # 3. Derivative term with low-pass filter
        if not self.is_first_run:
            raw_deriv = (error - self.last_error) / dt
            self.filtered_deriv = (self.deriv_alpha * raw_deriv) + ((1.0 - self.deriv_alpha) * self.filtered_deriv)
        else:
            self.filtered_deriv = 0.0
            self.is_first_run = False

        d_term = self.kd * self.filtered_deriv
        self.last_error = error

        total = p_term + i_term + d_term

        # 4. Output saturation
        if total > self.max_out:
            return float(self.max_out)
        elif total < self.min_out:
            return float(self.min_out)
        return total


def test_pid_proportional_response():
    pid = DiscretePID(kp=2.0, ki=0.0, kd=0.0, min_out=0, max_out=1023)
    # Target = 500, Measured = 450 -> error = 50 -> output = 2.0 * 50 = 100
    out = pid.compute(target=500, measured=450, dt=0.002)
    assert out == pytest.approx(100.0, rel=1e-3)


def test_pid_integral_anti_windup():
    pid = DiscretePID(kp=0.0, ki=1.0, kd=0.0, integral_min=-50.0, integral_max=50.0)
    # Run with large persistent error for 1000 steps
    for _ in range(1000):
        pid.compute(target=100, measured=0, dt=0.1)

    # Accumulator should strictly be clamped to integral_max
    assert pid.integral_sum == pytest.approx(50.0, rel=1e-3)


def test_pid_derivative_filtering():
    pid = DiscretePID(kp=0.0, ki=0.0, kd=1.0, deriv_alpha=0.5)
    # Step 1: Initialize
    pid.compute(target=100, measured=100, dt=0.01)
    assert pid.filtered_deriv == 0.0

    # Step 2: Sudden change
    pid.compute(target=100, measured=90, dt=0.01) # error jumps from 0 to 10 -> raw_deriv = 1000
    # filtered = 0.5 * 1000 + 0.5 * 0 = 500
    assert pid.filtered_deriv == pytest.approx(500.0, rel=1e-3)


def test_pid_output_saturation_limits():
    pid = DiscretePID(kp=100.0, ki=0.0, kd=0.0, min_out=0, max_out=870)
    # Very large positive error
    out_high = pid.compute(target=1000, measured=0, dt=0.01)
    assert out_high == 870.0

    # Negative error (object closer than setpoint)
    out_low = pid.compute(target=0, measured=500, dt=0.01)
    assert out_low == 0.0


def test_pid_bumpless_reset():
    pid = DiscretePID(kp=1.0, ki=1.0, kd=1.0)
    pid.compute(target=100, measured=50, dt=0.01)
    assert pid.integral_sum != 0.0
    assert pid.is_first_run is False

    pid.reset()
    assert pid.integral_sum == 0.0
    assert pid.last_error == 0.0
    assert pid.filtered_deriv == 0.0
    assert pid.is_first_run is True


if __name__ == "__main__":
    test_pid_proportional_response()
    test_pid_integral_anti_windup()
    test_pid_derivative_filtering()
    test_pid_output_saturation_limits()
    test_pid_bumpless_reset()
    print("All PID unit tests passed successfully!")
