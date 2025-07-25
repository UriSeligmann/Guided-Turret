import asyncio
import threading
import websockets

from motor_ops       import chunked_move, sync_move, stop_all, clear_stop, STOP_EVENT
from gpio_bus        import GPIOBus
from pinio           import PinIO
from stepper_motor   import load_motors

# ------------------ Hardware Init ------------------
bus    = GPIOBus("board.yaml")
io     = PinIO(bus)
motors = load_motors(io, "motors.yaml")
# Start each motor's thread so queue()/wait() work:
for m in motors.values():
    m.start()
motor_a = motors["motor_a"]
motor_b = motors["motor_b"]

# ------------------ Gear & Sync Constants ------------------
TEETH_A_DRIVER, TEETH_A_DRIVEN = 89, 71
TEETH_B_DRIVER, TEETH_B_DRIVEN = 20, 88
SYNC_RATIO = (TEETH_A_DRIVER/TEETH_A_DRIVEN) / (TEETH_B_DRIVER/TEETH_B_DRIVEN)
MIN_UNIT   = 10

# ------------------ Motion Thread State ------------------
motion_thread: threading.Thread = None

def _start_thread(target_fn):
    """
    Stop any prior motion, clear the stop-flag, then
    spawn a new daemon thread running target_fn().
    """
    global motion_thread
    stop_all()
    clear_stop()
    t = threading.Thread(target=target_fn, daemon=True)
    motion_thread = t
    t.start()

def handle_cmd(cmd: str) -> str:
    cmd = cmd.strip().lower()

    # STOP
    if cmd in ("x", "stop"):
        stop_all()
        return "stopped"

    # QUIT
    if cmd in ("q", "quit", "exit"):
        return "bye"

    # SYNC N
    if cmd.startswith("sync"):
        parts = cmd.split()
        if len(parts) != 2 or not parts[1].lstrip("-").isdigit():
            return "usage: sync N"
        n = int(parts[1])

        def run_sync():
            while not STOP_EVENT.is_set():
                sync_move(motor_a, motor_b, n, SYNC_RATIO, MIN_UNIT)

        _start_thread(run_sync)
        return f"sync start N={n}"

    # P N / Y N
    parts = cmd.split()
    if len(parts) == 2 and parts[0] in ("p", "y") and parts[1].lstrip("-").isdigit():
        axis, val = parts[0], int(parts[1])
        motor = motor_a if axis == "p" else motor_b
        direction = motor.io.bus.HIGH if val >= 0 else motor.io.bus.LOW
        step_count = abs(val)

        def run_axis():
            while not STOP_EVENT.is_set():
                chunked_move(motor, step_count, direction)

        _start_thread(run_axis)
        return f"{axis} start N={val}"

    return "? invalid. Use p N, y N, sync N, stop, or quit"


#WebSocket Handler (single-arg)
async def handler(ws):
    print(f"[+] Client connected: {ws.remote_address}")
    await ws.send("Connected!")
    try:
        async for msg in ws:
            print(f"[>] {msg.strip()}")
            resp = await asyncio.get_running_loop() \
                              .run_in_executor(None, handle_cmd, msg)
            print(f"[<] {resp}")
            await ws.send(resp)
            if resp == "bye":
                break
    except websockets.exceptions.ConnectionClosedOK:
        pass
    finally:
        print("[*] Handler cleanup")


# ??? Async Main & Clean Shutdown ???
async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("Motor server listening on ws://0.0.0.0:8765")
        # run forever until cancelled
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[!] KeyboardInterrupt received, shutting down")
    finally:
        # Abort any running motion
        stop_all()

        # Stop motors cleanly (skip dead threads)
        for m in motors.values():
            try:
                m.stop()
            except RuntimeError:
                pass

        bus.cleanup()
        print("[*] Clean exit.")
