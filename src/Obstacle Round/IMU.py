import serial
from time import time, sleep
from struct import unpack
from numpy import array

def get_acceleration(data):
    """Extracts acceleration data from the given byte data."""
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'Q':  # 0x51, expected command for acceleration data
        return array(unpack('<hhh', data[1:7])) / 32768.0 * 16.0
    return None

def get_gyro(data):
    """Extracts gyro data from the given byte data."""
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'R':  # 0x52, expected command for gyro data
        return array(unpack('<hhh', data[1:7])) / 32768.0 * 2000.0
    return None

def get_magnetic(data):
    """Extracts magnetic field data from the given byte data."""
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'T':  # 0x54, expected command for magnetic field data
        return array(unpack('<hhh', data[1:7]))
    return None

def get_angle(data):
    """Extracts angle data from the given byte data."""
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'S':  # 0x53, expected command for angle data
        return array(unpack('<hhh', data[1:7])) / 32768.0 * 180.0
    return None

def get_quaternion(data):
    """Extracts quaternion data from the given byte data."""
    if len(data) < 9:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'Y':  # 0x59, expected command for quaternion data
        q = array(unpack('<hhhh', data[1:9])) / 32768.0
        return array([q[1], q[2], q[3], q[0]])
    return None

class Imu():
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
        """Initializes the IMU with serial port and baudrate."""
        self.port = port
        self.baudrate = baudrate
        self.yaw = 0
        self.stop = False
        self.angularVelocities = []
        self.linearAccelerations = []
        self.noLinearAccelerationsCount = 0
        self.noAngularVelocitiesCount = 0

    def continuouslyReadData(self):
        """Continuously reads and processes data from the IMU."""
        while not self.stop:
            with serial.Serial(self.port, self.baudrate) as ser:
                while not self.stop:
                    message = ser.read_until(b'U')

                    # Process gyro data
                    g = get_gyro(message)
                    
                    if g is not None:
                        if (-2.4 < g[0] < 2.4 and -2.4 < g[1] < 2.4 and -2.4 < g[2] < 2.4):
                            self.noAngularVelocitiesCount += 1
                        else:
                            self.noAngularVelocitiesCount = 0
                        sleep(0.04)
                        ser.reset_input_buffer()
                        sleep(0.01)
                    else:
                        sleep(0.001)
