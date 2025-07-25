import threading
import time

# Global stop event when set, all motion loops will break out immediately.
STOP_EVENT = threading.Event()

def stop_all():
    """Trigger an emergency stop of any ongoing motion."""
    STOP_EVENT.set()

def clear_stop():
    """Clear the stop flag so that motion helpers can run again."""
    STOP_EVENT.clear()


# Overrideable from motors.yaml or elsewhere:
CHUNK_SIZE = 100


def chunked_move(motor, total_steps: int, direction: bool):
    """
    Move a single motor in chunks, but abort if stop_all() is called.
    """
    clear_stop()
    rem = total_steps
    while rem and not STOP_EVENT.is_set():
        chunk = min(CHUNK_SIZE, rem)
        motor.queue(chunk, forward=(direction == motor.io.bus.HIGH))
        motor.wait()
        rem -= chunk
    if STOP_EVENT.is_set():
        return


def chunked_dual_move(motor_a, motor_b, steps_a: int, steps_b: int):
    """
    Move two motors independently in chunks, but abort both if stop_all() is called.
    """
    clear_stop()
    rem_a, rem_b = abs(steps_a), abs(steps_b)
    dir_a = motor_a.io.bus.HIGH if steps_a >= 0 else motor_a.io.bus.LOW
    dir_b = motor_b.io.bus.HIGH if steps_b >= 0 else motor_b.io.bus.LOW

    while (rem_a or rem_b) and not STOP_EVENT.is_set():
        ca = min(CHUNK_SIZE, rem_a)
        cb = min(CHUNK_SIZE, rem_b)
        if ca:
            motor_a.queue(ca, forward=(dir_a == motor_a.io.bus.HIGH))
        if cb:
            motor_b.queue(cb, forward=(dir_b == motor_b.io.bus.HIGH))
        if ca:
            motor_a.wait()
        if cb:
            motor_b.wait()
        rem_a -= ca
        rem_b -= cb

    if STOP_EVENT.is_set():
        return


def sync_move(motor_a, motor_b, steps: int, sync_ratio: float, minimal_motion_unit: int):
    """
    Drive two motors so that motor_b moves in sync with motor_a according
    to sync_ratio, but abort immediately if stop_all() is called.
    """
    clear_stop()
    total_iterations = round(abs(steps) / minimal_motion_unit)
    direction = steps / abs(steps)

    if sync_ratio < 1:
        sync_ratio = 1.0 / sync_ratio

    for _ in range(total_iterations):
        if STOP_EVENT.is_set():
            break

        # compute each slice
        steps_b = minimal_motion_unit * direction
        steps_a = -round(steps_b * sync_ratio)

        dir_b = motor_b.io.bus.HIGH if steps_a > 0 else motor_b.io.bus.LOW
        dir_a = motor_a.io.bus.LOW  if steps_b > 0 else motor_a.io.bus.HIGH

        rem_a = abs(steps_a)
        rem_b = abs(steps_b)

        # queue them
        if rem_b:
            motor_b.queue(rem_b, forward=(dir_b == motor_b.io.bus.HIGH))
        if rem_a:
            motor_a.queue(rem_a, forward=(dir_a == motor_a.io.bus.HIGH))

        if STOP_EVENT.is_set():
            break

        # cross wait
        if rem_a:
            motor_b.wait()
        if rem_b:
            motor_a.wait()

    if STOP_EVENT.is_set():
        return
