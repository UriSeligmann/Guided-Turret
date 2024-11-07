import smbus2
import struct
import threading

class I2CCommunicator:
    def __init__(self, address, bus_number=1):
        """
        Initializes the I2C communicator.

        :param address: The I2C address of the slave device (Arduino).
        :param bus_number: The I2C bus number (default is 1 for Raspberry Pi 3 and later).
        """
        self.address = address
        self.bus_number = bus_number
        self.lock = threading.Lock()  # To ensure thread-safe access to the I2C bus

    def write_floats(self, floats):
        """
        Sends a list of two floats to the slave device.

        :param floats: A list or tuple containing two float values.
        """
        # Pack the float values into bytes (little-endian format)
        data = struct.pack('<ff', *floats)
        # Create an I2C write message
        write = smbus2.i2c_msg.write(self.address, data)
        # Send the data to the Arduino with thread safety
        with self.lock:
            with smbus2.SMBus(self.bus_number) as bus:
                bus.i2c_rdwr(write)

    def read_floats(self):
        """
        Reads two floats from the slave device.

        :return: A tuple containing two float values.
        """
        # Create an I2C read message
        read = smbus2.i2c_msg.read(self.address, 8)
        # Read data from the Arduino with thread safety
        with self.lock:
            with smbus2.SMBus(self.bus_number) as bus:
                bus.i2c_rdwr(read)
                data = bytes(read)
        # Unpack the bytes into float values
        floats = struct.unpack('<ff', data)
        return floats