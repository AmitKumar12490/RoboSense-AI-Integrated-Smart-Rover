import pytest

class DiscreteSensorCalibration:
    """Exact python mirror of firmware/src/maglev/SensorCalibration.cpp"""
    def __init__(self, raw_min=60, raw_max=4035, alpha=0.30):
        self.raw_min = raw_min
        self.raw_max = raw_max
        self.alpha = alpha
        self.baseline = 2048.0
        self.filtered_pos = 0.0
        self.is_calibrated = False

    def is_sensor_valid(self, raw: int) -> bool:
        return self.raw_min <= raw <= self.raw_max

    def calibrate(self, samples: list) -> bool:
        valid_samples = [s for s in samples if self.is_sensor_valid(s)]
        if len(valid_samples) < (len(samples) * 0.75):
            self.is_calibrated = False
            return False
        self.baseline = sum(valid_samples) / len(valid_samples)
        self.filtered_pos = 0.0
        self.is_calibrated = True
        return True

    def update(self, raw: int) -> float:
        if not self.is_sensor_valid(raw):
            return self.filtered_pos
        relative = float(raw) - self.baseline
        self.filtered_pos = (self.alpha * relative) + ((1.0 - self.alpha) * self.filtered_pos)
        return self.filtered_pos


def test_sensor_validity_bounds():
    cal = DiscreteSensorCalibration(raw_min=60, raw_max=4035)
    assert cal.is_sensor_valid(2048) is True
    assert cal.is_sensor_valid(50) is False   # Disconnected / open pin
    assert cal.is_sensor_valid(4040) is False # Rail short / saturated


def test_calibration_tare_routine():
    cal = DiscreteSensorCalibration()
    # 100 samples around 2100 ADC units
    samples = [2100 + (i % 5) for i in range(100)]
    success = cal.calibrate(samples)
    assert success is True
    assert cal.is_calibrated is True
    assert 2098.0 <= cal.baseline <= 2104.0


def test_calibration_rejection_on_noisy_disconnected_sensor():
    cal = DiscreteSensorCalibration()
    # Mostly invalid samples
    samples = [10] * 80 + [2048] * 20
    success = cal.calibrate(samples)
    assert success is False
    assert cal.is_calibrated is False


def test_exponential_filter_smoothing():
    cal = DiscreteSensorCalibration(alpha=0.25)
    cal.baseline = 2000.0

    # Step input: raw reading jumps to 2100 (relative = +100)
    pos1 = cal.update(2100)
    assert pos1 == pytest.approx(25.0, rel=1e-2)

    pos2 = cal.update(2100)
    assert pos2 == pytest.approx(43.75, rel=1e-2)


if __name__ == "__main__":
    test_sensor_validity_bounds()
    test_calibration_tare_routine()
    test_calibration_rejection_on_noisy_disconnected_sensor()
    test_exponential_filter_smoothing()
    print("All sensor calibration tests passed successfully!")
