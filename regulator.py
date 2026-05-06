import time

class Regulator:
    def __init__(self, kp=0.12):
        self.kp = kp
        self.target_angle = 0

    def update(self, error):
        self.target_angle = error * self.kp
        return max(min(self.target_angle, 90), -90)

class KalmanLite:
    def __init__(self, process_noise=0.05, measurement_noise=2.0):
        self.q = process_noise
        self.r = measurement_noise
        self.x = None
        self.p = 1.0

    def apply(self, measurement):
        if self.x is None:
            self.x = measurement
            return measurement
        
        self.p = self.p + self.q
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * self.p
        return self.x