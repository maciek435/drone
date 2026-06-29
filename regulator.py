class Regulator:
    def __init__(self, kp=0.12, kd=0.1, max_output=50):
        self.kp = kp
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0

    def update(self, error):
        derivative = error - self.prev_error
        self.prev_error = error
        output = (error * self.kp) + (derivative * self.kd)
        return max(min(output, self.max_output), -self.max_output)

    def reset(self):
        self.prev_error = 0

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

    def reset(self):
        self.x = None
        self.p = 1.0

class DistanceRegulator:
    def __init__(self, kp=0.2, kd=0.05, max_jump=15, max_output=50):
        self.kp = kp
        self.kd = kd               
        self.max_jump = max_jump
        self.max_output = max_output
        self.target_height = None
        self.last_height = None
        self.prev_error = 0         

    def set_reference(self, current_height):
        self.target_height = current_height
        self.last_height = current_height
        self.prev_error = 0

    def update(self, current_height):
        if self.target_height is None:
            return 0
        if current_height <= 0:
            return 0

        if self.last_height is not None:
            jump = abs(current_height - self.last_height)
            if jump > self.max_jump:
                return 0

        self.last_height = current_height
        error = self.target_height - current_height

        derivative = error - self.prev_error
        self.prev_error = error

        output = (error * self.kp) + (derivative * self.kd)
        return max(min(output, self.max_output), -self.max_output)

    def reset(self):
        self.prev_error = 0
        self.last_height = None