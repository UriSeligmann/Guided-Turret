import time

class PinIO:
    """Layer 2.  Tiny stateless helpers: pulse() and set()."""
    def __init__(self, bus): self.bus = bus

    def pulse(self, pin: int, half_period_us: int):
        hp = half_period_us / 1_000_000.0
        self.bus.write(pin, self.bus.HIGH)
        time.sleep(hp)
        self.bus.write(pin, self.bus.LOW)
        time.sleep(hp)

    def set(self, pin: int, high: bool):
        self.bus.write(pin, self.bus.HIGH if high else self.bus.LOW)
