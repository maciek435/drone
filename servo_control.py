from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

class ServoController:
    def __init__(self, pin=17):
        factory = PiGPIOFactory()
        self.servo = AngularServo(pin, min_angle=-90, max_angle=90, pin_factory=factory)
        self.current_angle = 0
        self.servo.angle = 0

    def move_smoothly(self, target_angle, speed=0.15):
        # Liniowa interpolacja (Lerp)
        diff = target_angle - self.current_angle
        self.current_angle += diff * speed
        
        # Zabezpieczenie zakresu
        self.current_angle = max(min(self.current_angle, 90), -90)
        self.servo.angle = self.current_angle