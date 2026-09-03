# HC-SR04+ (3.3V) distance sensors - left, front, right.
from machine import Pin, time_pulse_us
import time
import config

_TIMEOUT_US = 30000     # ~5 m, beyond that we report "no wall"
_NO_ECHO_MM = 4000


class Sonar:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.value(0)

    def read_mm(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        try:
            us = time_pulse_us(self.echo, 1, _TIMEOUT_US)
        except OSError:
            return _NO_ECHO_MM
        if us < 0:
            return _NO_ECHO_MM
        return int(us * 0.1715)     # 343 m/s, there and back


class SonarArray:
    def __init__(self):
        self.left = Sonar(config.PIN_SONAR_L_TRIG, config.PIN_SONAR_L_ECHO)
        self.front = Sonar(config.PIN_SONAR_F_TRIG, config.PIN_SONAR_F_ECHO)
        self.right = Sonar(config.PIN_SONAR_R_TRIG, config.PIN_SONAR_R_ECHO)

    def read_all(self):
        return {
            "left": self.left.read_mm(),
            "front": self.front.read_mm(),
            "right": self.right.read_mm(),
        }

    def walls(self):
        """-> dict of side -> bool, using WALL_THRESHOLD_MM."""
        d = self.read_all()
        t = config.WALL_THRESHOLD_MM
        return {k: (v < t) for k, v in d.items()}
