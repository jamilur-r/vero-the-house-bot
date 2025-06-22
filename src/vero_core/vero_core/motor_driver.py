# vero_core/motor_driver.py

from vero_core.pca import PCA9685
import time

GLOBAL_SPEED = 60
class MotorDriver:
    def __init__(self, address=0x5f):
        self.pwm = PCA9685(address)
        self.pwm.setPWMFreq(50)

        # Motor A
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2

        # Motor B
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    # --- Motor A Control ---
    def motor_a(self, direction, speed=GLOBAL_SPEED):
        self.pwm.setDutycycle(self.PWMA, speed)
        if direction == 'forward':
            self.pwm.setLevel(self.AIN1, 1)
            self.pwm.setLevel(self.AIN2, 0)
        elif direction == 'backward':
            self.pwm.setLevel(self.AIN1, 0)
            self.pwm.setLevel(self.AIN2, 1)
        else:
            self.pwm.setLevel(self.AIN1, 0)
            self.pwm.setLevel(self.AIN2, 0)
            self.pwm.setDutycycle(self.PWMA, 0)

    # --- Motor B Control ---
    def motor_b(self, direction, speed=GLOBAL_SPEED):
        self.pwm.setDutycycle(self.PWMB, speed)
        if direction == 'forward':
            self.pwm.setLevel(self.BIN1, 1)
            self.pwm.setLevel(self.BIN2, 0)
        elif direction == 'backward':
            self.pwm.setLevel(self.BIN1, 0)
            self.pwm.setLevel(self.BIN2, 1)
        else:
            self.pwm.setLevel(self.BIN1, 0)
            self.pwm.setLevel(self.BIN2, 0)
            self.pwm.setDutycycle(self.PWMB, 0)

    # --- Combined Motion ---
    def move_forward(self, speed=GLOBAL_SPEED):
        print("🟢 Forward")
        self.motor_a('forward', speed)
        self.motor_b('forward', speed)

    def move_backward(self, speed=GLOBAL_SPEED):
        print("🔴 Backward")
        self.motor_a('backward', speed)
        self.motor_b('backward', speed)

    def turn_left(self, speed=GLOBAL_SPEED):
        print("↩️ Turning left (spin in place)")
        self.motor_a('backward', speed)
        self.motor_b('forward', speed)

    def turn_right(self, speed=GLOBAL_SPEED):
        print("↪️ Turning right (spin in place)")
        self.motor_a('forward', speed)
        self.motor_b('backward', speed)

    def stop(self):
        print("⛔ Stopping")
        self.motor_a('stop')
        self.motor_b('stop')
