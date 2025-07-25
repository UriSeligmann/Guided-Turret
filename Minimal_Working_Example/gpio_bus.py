import yaml, RPi.GPIO as GPIO
from pathlib import Path

DIR = {"in": GPIO.IN, "out": GPIO.OUT}

class GPIOBus:
    """
    Layer 1.  Reads board.yaml, sets mode, exports two helpers:
      - write(pin, level)
      - read(pin)           (for completeness)
    All declared pins are forced LOW at start-up.
    """
    def __init__(self, yaml_path="board.yaml"):
        cfg  = yaml.safe_load(Path(yaml_path).read_text())
        mode = cfg.get("mode", "BCM").upper()
        GPIO.setmode(GPIO.BCM if mode == "BCM" else GPIO.BOARD)

        self.HIGH, self.LOW = GPIO.HIGH, GPIO.LOW
        self._pins = {}                     # {int pin#: dir str}

        for pin_str, dir_str in cfg["pins"].items():
            pin, d = int(pin_str), dir_str.lower()
            if d not in DIR:
                raise ValueError(f"Pin {pin}: dir must be 'in' or 'out'")
            GPIO.setup(pin, DIR[d], initial=GPIO.LOW)
            self._pins[pin] = d

    # ---------- public API -----------------------------------------
    def assert_known(self, pin: int):
        if pin not in self._pins:
            raise KeyError(f"Pin {pin} not declared in board.yaml")
        return pin

    def write(self, pin: int, level):
        self.assert_known(pin)
        GPIO.output(pin, level)

    def read(self, pin: int):
        self.assert_known(pin)
        return GPIO.input(pin)

    def cleanup(self): GPIO.cleanup()
