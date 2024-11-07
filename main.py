# main.py
import threading
import time
from i2c_comm import I2CCommunicator
import queue

# Function to be run in a separate thread
def arduino_communication_thread(communicator, float_queue, stop_event):
    """
    Thread function for sending and receiving floats from the queue.

    :param communicator: An instance of I2CCommunicator.
    :param float_queue: A queue from which to read float pairs to send.
    :param stop_event: An event to signal the thread to stop.
    """
    while not stop_event.is_set():
        try:
            # Wait for a new float pair to send
            floats_to_send = float_queue.get(timeout=0.1)
            print(f"{threading.current_thread().name} sending floats: {floats_to_send}")
            communicator.write_floats(floats_to_send)
            # Wait for the Arduino to process the data
            time.sleep(0.1)
            received_floats = communicator.read_floats()
            print(f"{threading.current_thread().name} received floats: {received_floats}")
            float_queue.task_done()
        except queue.Empty:
            continue
    print(f"{threading.current_thread().name} has been signaled to stop.")

if __name__ == '__main__':
    SLAVE_ADDRESS = 0x08  # I2C address of the Arduino
    communicator = I2CCommunicator(SLAVE_ADDRESS)

    float_queue = queue.Queue()
    stop_event = threading.Event()

    # Create and start the communication thread
    arduino_comm_thread = threading.Thread(
        target=arduino_communication_thread,
        args=(communicator, float_queue, stop_event),
        name="I2C-Communication-Thread"
    )
    arduino_comm_thread.start()

    # Dynamically send float pairs to the communication thread
    try:
        while True:
            # Simulate getting new float pairs dynamically
            float_pair = input("Enter two floats separated by space (or 'exit' to quit): ")
            if float_pair.strip().lower() == 'exit':
                break
            floats = tuple(map(float, float_pair.strip().split()))
            if len(floats) != 2:
                print("Please enter exactly two float numbers.")
                continue
            float_queue.put(floats)
    except KeyboardInterrupt:
        pass
    finally:
        # Signal the communication thread to stop
        stop_event.set()
        # Wait for the queue to be empty
        float_queue.join()
        # Wait for the thread to finish
        arduino_comm_thread.join()
        print("Communication thread has been stopped.")