from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

class ServoGimbal:
    def __init__(self, pin, min_angle=-60, max_angle=60, reversed=False):
        factory = PiGPIOFactory()
        self.servo = AngularServo(pin, min_angle=-90, max_angle=90, pin_factory=factory)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.reversed = reversed
        self.current_angle = 0
        self.servo.angle = 0

    def set_offset(self, offset, speed=0.15):
        if self.reversed:
            offset = -offset
        target = max(self.min_angle, min(self.max_angle, offset))

        diff = target - self.current_angle
        self.current_angle += diff * speed
        self.current_angle = max(min(self.current_angle, self.max_angle), self.min_angle)
        self.servo.angle = self.current_angle

    def stop(self):
        self.servo.detach()