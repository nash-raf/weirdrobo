# Quadrature encoders, x4 decoding via pin-change IRQs on both channels.
from machine import Pin
import config

_TRANS = {
    0b0001: 1,  0b0111: 1,  0b1110: 1,  0b1000: 1,
    0b0010: -1, 0b0100: -1, 0b1101: -1, 0b1011: -1,
}


class Encoder:
    def __init__(self, pin_a, pin_b):
        self.a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.count = 0
        self._prev = (self.a.value() << 1) | self.b.value()
        trig = Pin.IRQ_RISING | Pin.IRQ_FALLING
        self.a.irq(trigger=trig, handler=self._irq)
        self.b.irq(trigger=trig, handler=self._irq)

    def _irq(self, _pin):
        cur = (self.a.value() << 1) | self.b.value()
        self.count += _TRANS.get((self._prev << 2) | cur, 0)
        self._prev = cur

    def reset(self):
        self.count = 0

    def mm(self):
        return self.count * config.MM_PER_COUNT


class EncoderPair:
    def __init__(self):
        self.left = Encoder(config.PIN_ENCL_A, config.PIN_ENCL_B)
        self.right = Encoder(config.PIN_ENCR_A, config.PIN_ENCR_B)

    def reset(self):
        self.left.reset()
        self.right.reset()

    def counts(self):
        return (self.left.count, self.right.count)

    def average(self):
        return (abs(self.left.count) + abs(self.right.count)) / 2
