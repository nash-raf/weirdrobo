# Sensor and actuator drivers for the Pico W.
#
# Adapted from the firmware/ drivers on samiulislam07/autonomous-maze-solving-
# robot @ Samiul, re-pointed at this project's measured calibration in
# config.py. MicroPython only - `machine` does not exist on a laptop, so this
# module is imported on the robot and never by the desktop tests.
#
# Everything here is deliberately dumb: read a distance, count a tick, set a
# duty. Deciding what to do with any of it belongs in motion.py and runner.py.

from machine import PWM, Pin, time_pulse_us
from utime import sleep_us, ticks_diff, ticks_ms

import config


class Ultrasonic:
    """One HC-SR04+.

    read_mm() blocks for as long as the echo takes - up to ~30 ms. That is why
    the run loop only reads while the robot is stopped at a cell: a blocking
    read mid-move would also stall the web server.
    """

    def __init__(self, trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trigger.value(0)

    def read_once_mm(self):
        """One ping. None when nothing came back in time."""
        self.trigger.value(0)
        sleep_us(5)
        self.trigger.value(1)
        sleep_us(10)
        self.trigger.value(0)
        try:
            duration = time_pulse_us(self.echo, 1, config.ULTRASONIC_TIMEOUT_US)
        except OSError:
            return None
        if duration < 0:
            return None
        return duration * config.SPEED_OF_SOUND_MM_PER_US / 2.0   # out and back

    def read_mm(self, samples=None):
        """Median of several pings, so one dropped echo cannot invent a wall.

        Returns a large distance rather than None when every ping fails: an
        unreadable sensor should look like open space to the mapper and be
        caught by the operator, not silently freeze the robot mid-maze.
        """
        count = samples or config.SENSOR_SAMPLES
        readings = []
        for _ in range(count):
            value = self.read_once_mm()
            if value is not None:
                readings.append(value)
            sleep_us(6000)      # let the echo of the previous ping die away
        if not readings:
            return config.NO_ECHO_MM
        readings.sort()
        return readings[len(readings) // 2]


# x4 quadrature: both edges of both channels. This is what the measured
# CPR = 5887 was taken with, so the decode and the constant must stay together.
_TRANS = {
    0b0001: 1,  0b0111: 1,  0b1110: 1,  0b1000: 1,
    0b0010: -1, 0b0100: -1, 0b1101: -1, 0b1011: -1,
}


class Encoder:
    def __init__(self, pin_a, pin_b, inverted=False):
        self.a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.inverted = inverted
        self.ticks = 0
        self._prev = (self.a.value() << 1) | self.b.value()
        trig = Pin.IRQ_RISING | Pin.IRQ_FALLING
        self.a.irq(trigger=trig, handler=self._irq)
        self.b.irq(trigger=trig, handler=self._irq)

    def _irq(self, _pin):
        cur = (self.a.value() << 1) | self.b.value()
        step = _TRANS.get((self._prev << 2) | cur, 0)
        self.ticks += -step if self.inverted else step
        self._prev = cur

    def reset(self):
        self.ticks = 0

    def mm(self):
        return self.ticks * config.MM_PER_COUNT


class Motor:
    """One TB6612FNG channel: two direction pins and one PWM pin."""

    def __init__(self, pwm_pin, in1_pin, in2_pin, inverted=False):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(config.PWM_FREQ)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.inverted = inverted
        self.stop()

    def drive(self, duty):
        """duty in [-1.0, 1.0], clamped to MAX_PWM in both directions."""
        if duty > config.MAX_PWM:
            duty = config.MAX_PWM
        elif duty < -config.MAX_PWM:
            duty = -config.MAX_PWM
        if self.inverted:
            duty = -duty
        forward = duty >= 0
        self.in1.value(1 if forward else 0)
        self.in2.value(0 if forward else 1)
        self.pwm.duty_u16(int(abs(duty) * 65535))

    def stop(self):
        """Short brake: both direction pins low, zero duty."""
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty_u16(0)


class Robot:
    """Everything bolted to the chassis, wired up once."""

    def __init__(self):
        check_pins()

        self.front = Ultrasonic(config.PIN_SONAR_F_TRIG, config.PIN_SONAR_F_ECHO)
        self.left_sensor = Ultrasonic(config.PIN_SONAR_L_TRIG, config.PIN_SONAR_L_ECHO)
        self.right_sensor = Ultrasonic(config.PIN_SONAR_R_TRIG, config.PIN_SONAR_R_ECHO)

        self.standby = Pin(config.PIN_STBY, Pin.OUT)
        self.standby.value(0)       # driver disabled until something asks to move

        self.left_motor = Motor(config.PIN_PWMA, config.PIN_AIN1, config.PIN_AIN2,
                                config.LEFT_MOTOR_INVERTED)
        self.right_motor = Motor(config.PIN_PWMB, config.PIN_BIN1, config.PIN_BIN2,
                                 config.RIGHT_MOTOR_INVERTED)
        self.left_encoder = Encoder(config.PIN_ENCL_A, config.PIN_ENCL_B,
                                    config.LEFT_MOTOR_INVERTED)
        self.right_encoder = Encoder(config.PIN_ENCR_A, config.PIN_ENCR_B,
                                     config.RIGHT_MOTOR_INVERTED)

    # -- sensing --------------------------------------------------------
    def read_distances(self):
        """Front, left, right in mm. Only call this while stopped."""
        return (self.front.read_mm(),
                self.left_sensor.read_mm(),
                self.right_sensor.read_mm())

    def walls(self):
        """-> dict side -> bool, relative to the robot, not the maze."""
        front, left, right = self.read_distances()
        t = config.WALL_THRESHOLD_MM
        return {"front": front < t, "left": left < t, "right": right < t}

    def encoder_counts(self):
        return self.left_encoder.ticks, self.right_encoder.ticks

    def reset_encoders(self):
        self.left_encoder.reset()
        self.right_encoder.reset()

    # -- actuation ------------------------------------------------------
    def enable(self):
        self.standby.value(1)

    def drive(self, left_duty, right_duty):
        self.enable()
        self.left_motor.drive(left_duty)
        self.right_motor.drive(right_duty)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.standby.value(0)

    def emergency_stop(self):
        """Cut drive at the driver, not just the duty cycle.

        Pulling STBY low disables both H-bridges regardless of what the PWM
        peripheral is still doing, so this stays effective even if the control
        loop above it has wedged.
        """
        self.standby.value(0)
        self.left_motor.stop()
        self.right_motor.stop()


def check_pins():
    """Raise if two signals claim the same GPIO, or one is out of range.

    Cheap insurance against a typo in config.py. Called before any motor moves.
    """
    names = [n for n in dir(config) if n.startswith("PIN_")]
    seen = {}
    for name in names:
        pin = getattr(config, name)
        if not isinstance(pin, int) or not 0 <= pin <= 28:
            raise ValueError("Pin " + name + " is not a valid Pico GPIO: " + str(pin))
        if pin in seen:
            raise ValueError("GPIO " + str(pin) + " is claimed by both "
                             + seen[pin] + " and " + name)
        seen[pin] = name
    return True


def elapsed_ms(since):
    return ticks_diff(ticks_ms(), since)


def now_ms():
    return ticks_ms()
