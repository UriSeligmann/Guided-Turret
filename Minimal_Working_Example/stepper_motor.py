import threading, yaml, time
from pathlib import Path

class StepperMotor(threading.Thread):
    def __init__(self, pinio, *, pulse_pin, dir_pin,
                 delay_us, name=None, chunk_size=100):
        super().__init__(daemon=True, name=name or f"Motor-{pulse_pin}")
        # validate against board layer
        pinio.bus.assert_known(pulse_pin)
        pinio.bus.assert_known(dir_pin)

        self.io = pinio
        self.pulse_pin, self.dir_pin = pulse_pin, dir_pin
        self.half_period_us = delay_us
        self.CHUNK = chunk_size

        self._todo = 0
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._done_evt = threading.Event()

    # threaded loop & helpers ------------------------------------------------
    def run(self):
        while not self._stop_evt.is_set():
            with self._lock:
                todo = self._todo
            if todo:
                self.io.pulse(self.pulse_pin, self.half_period_us)
                with self._lock:
                    self._todo -= 1
                    if self._todo == 0:
                        self._done_evt.set()
            else:
                time.sleep(0.001)

    def queue(self, steps: int, forward: bool):
        self.io.set(self.dir_pin, forward)
        with self._lock:
            self._todo = steps
            self._done_evt.clear()

    def wait(self): self._done_evt.wait()
    def stop(self):
        self._stop_evt.set(); self.join()

# factory ---------------------------------------------------------------
def load_motors(pinio, yaml_path="motors.yaml"):
    cfg = yaml.safe_load(Path(yaml_path).read_text())
    motors = {}
    for name, spec in cfg["stepper_motors"].items():
        motors[name] = StepperMotor(
            pinio,
            pulse_pin = spec["pulse_pin"],
            dir_pin   = spec["dir_pin"],
            delay_us  = spec["delay_us"],
            name      = name,
            chunk_size=cfg.get("defaults", {}).get("chunk_size", 100),
        )
    return motors
