import time

class EMAFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.smoothed_value = None

    def apply(self, current_value):
        if self.smoothed_value is None:
            self.smoothed_value = current_value
        else:
            self.smoothed_value = (self.alpha * current_value) + (1 - self.alpha) * self.smoothed_value
        return self.smoothed_value

class PID:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, current_value):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0 : dt = 0.001

        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        self.last_time = now
        return output