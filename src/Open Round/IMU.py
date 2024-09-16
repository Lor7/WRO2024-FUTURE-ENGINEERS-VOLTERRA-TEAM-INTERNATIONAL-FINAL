import serial
from time import sleep
from struct import unpack
from numpy import array
from threading import Thread
import matplotlib.pyplot as plt

def get_acceleration(data):
    """
    Extracts acceleration data from the received bytes.
    
    Parameters:
    data (bytes): The byte data received from the IMU.
    
    Returns:
    numpy.ndarray: The acceleration values in X, Y, Z directions.
    """
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'Q':  # 0x51 for acceleration
        return array(unpack('<hhh', data[1:7])) / 32768.0 * 16.0
    return None

def get_gyro(data):
    """
    Extracts gyroscope data from the received bytes.
    
    Parameters:
    data (bytes): The byte data received from the IMU.
    
    Returns:
    numpy.ndarray: The gyroscope values in X, Y, Z directions.
    """
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'R':  # 0x52 for gyroscope
        return array(unpack('<hhh', data[1:7])) / 32768.0 * 2000.0
    return None

def get_magnetic(data):
    """
    Extracts magnetic field data from the received bytes.
    
    Parameters:
    data (bytes): The byte data received from the IMU.
    
    Returns:
    numpy.ndarray: The magnetic field values in X, Y, Z directions.
    """
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'T':  # 0x54 for magnetic
        return array(unpack('<hhh', data[1:7]))
    return None

def get_angle(data):
    """
    Extracts angle data from the received bytes.
    
    Parameters:
    data (bytes): The byte data received from the IMU.
    
    Returns:
    numpy.ndarray: The angle values in X, Y, Z directions.
    """
    if len(data) < 7:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'S':  # 0x53 for angle
        return array(unpack('<hhh', data[1:7])) / 32768.0 * 180.0
    return None

def get_quaternion(data):
    """
    Extracts quaternion data from the received bytes.
    
    Parameters:
    data (bytes): The byte data received from the IMU.
    
    Returns:
    numpy.ndarray: The quaternion values.
    """
    if len(data) < 9:
        return None
    cmd = unpack('c', data[0:1])[0]
    if cmd == b'Y':  # 0x59 for quaternion
        q = array(unpack('<hhhh', data[1:9])) / 32768.0
        return array([q[1], q[2], q[3], q[0]])
    return None

class Imu:
    
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.yaw = 0
        self.stop = False
        self.angularVelocities = []
        self.linearAccelerations = []
        self.noLinearAccelerationsCount = 0
        self.noAngularVelocitiesCount = 0
        
    def continuouslyReadData(self):
        """
        Continuously reads data from the IMU and processes it.
        """
        with serial.Serial(self.port, self.baudrate) as ser:
            while not self.stop:
                message = ser.read_until(b'U')
                g = get_gyro(message)
                # a2 = get_acceleration(message)
                
                if g is not None:
                    if (-2.4 < g[0] < 2.4 and -2.4 < g[1] < 2.4 and -2.4 < g[2] < 2.4):
                        self.noAngularVelocitiesCount += 1
                    else:
                        self.noAngularVelocitiesCount = 0
                    sleep(0.04)
                    ser.reset_input_buffer()
                    sleep(0.01)
                
                # if a2 is not None:
                #     if (-0.05 < a2[0] < 0.05 and -0.05 < a2[1] < 0.05):
                #         self.noLinearAccelerationsCount += 1
                #     else:
                #         self.noLinearAccelerationsCount = 0
                else:
                    sleep(0.001)