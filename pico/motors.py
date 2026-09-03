# TB6612FNG dual H-bridge driver.
from machine import Pin, PWM
import config

_PWM_FREQ = 20000
_MAX = 65535


class Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin, inverted=False):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(_PWM_FREQ)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.inverted = inverted
        self.stop()

    def drive(self, duty):
        """duty in [-1.0, 1.0]. Positive = forward."""
        if self.inverted:
            duty = -duty
        if duty > 1.0:
            duty = 1.0
        elif duty < -1.0:
            duty = -1.0
        self.in1.value(1 if duty > 0 else 0)
        self.in2.value(1 if duty < 0 else 0)
        self.pwm.duty_u16(int(abs(duty) * _MAX))

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty_u16(0)


class Drive:
    def __init__(self):
        self.stby = Pin(config.PIN_STBY, Pin.OUT)
        self.left = Motor(config.PIN_PWMA, config.PIN_AIN1, config.PIN_AIN2,
                          inverted=config.LEFT_MOTOR_INVERTED)
        self.right = Motor(config.PIN_PWMB, config.PIN_BIN1, config.PIN_BIN2)
        self.enable()

    def enable(self):
        self.stby.value(1)

    def disable(self):
        self.stop()
        self.stby.value(0)

    def set(self, left_duty, right_duty):
        self.left.drive(left_duty)
        self.right.drive(right_duty)

    def stop(self):
        self.left.stop()
        self.right.stop()
