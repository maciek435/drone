import time
import board
import digitalio
import adafruit_vl53l1x

class ToFArray:
    def __init__(self, xshut_pins, addresses, distance_mode=2, timing_budget=200):
        self.i2c = board.I2C()
        self.xshut = [digitalio.DigitalInOut(pin) for pin in xshut_pins]

        for shut in self.xshut:
            shut.switch_to_output(value=False)
        time.sleep(0.2)

        self.sensors = []
        for i, shut in enumerate(self.xshut):
            shut.value = True
            time.sleep(0.2)
            sensor = adafruit_vl53l1x.VL53L1X(self.i2c)
            sensor.set_address(addresses[i])
            sensor.distance_mode = distance_mode
            sensor.timing_budget = timing_budget
            sensor.start_ranging()
            self.sensors.append(sensor)

    def read_all_cm(self):
        """Zwraca (center, left, right) w cm, None jesli brak/niewiarygodny odczyt."""
        results = []
        for sensor in self.sensors:
            try:
                if sensor.data_ready:
                    dist_cm = sensor.distance  # JUZ w cm
                    sensor.clear_interrupt()
                    if dist_cm is not None and 0 < dist_cm <= 400:
                        results.append(dist_cm)
                    else:
                        results.append(None)
                else:
                    results.append(None)
            except Exception:
                results.append(None)
        return tuple(results)

    def stop(self):
        for sensor in self.sensors:
            try:
                sensor.stop_ranging()
            except Exception:
                pass