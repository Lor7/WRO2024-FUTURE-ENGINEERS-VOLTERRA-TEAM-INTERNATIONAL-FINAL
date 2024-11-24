from numpy import array, uint8, pi
from colorama import Fore, Style
from colorSensor import ColorSensor
from gpiozero import Motor, AngularServo
from threading import Thread
from time import sleep

# Initialize threads and image processing variables
colorThread, defineColorThread = None, None
imageProcess = None

# Global variable to hold the color sensor object
colorSensor = None

def initialize_color_sensor():
    """
    Initialize the color sensor by continuously attempting to create an instance
    until successful. This ensures the color sensor is properly set up before use.
    """
    global colorSensor
    while colorSensor is None:
        try:
            colorSensor = ColorSensor()  # Attempt to create the color sensor instance
        except Exception as e:
            sleep(0.02)  # Wait briefly before retrying if an error occurs

# Create and start a thread to initialize the color sensor
color_sensor_thread = Thread(target=initialize_color_sensor)
color_sensor_thread.start()
color_sensor_thread.join()  # Wait for the color sensor thread to complete

# Steering and servo configuration
STEERINGANGLEMID = 0  # Midpoint angle for steering (used for reference)
SERVOOFFSET = +50     # Offset for the servo angle to adjust its position

# Initialize motor with GPIO pins and PWM settings
motor = Motor(forward=24,
            backward=23,
            enable=18,
            pwm=True,
            pin_factory=None)

# Initialize servo with GPIO pins, pulse width, and angle settings
servo = AngularServo(pin=17,
                    initial_angle=2,
                    min_angle=-90,
                    max_angle=90,
                    min_pulse_width=0.0006,
                    max_pulse_width=0.0024,
                    pin_factory=None)

# Global Variables
# -----------------
# Variables for tracking the state and control parameters
# Counters and flags for lap and side detection
lap = 0
side = 0
updatedSide = 0
# Position, angle, speed, and motor value settings
pos = None
angle = None
speed = 0.4
motorValue = 0.235
colorDict = {'red': None, 'green': None, 'blue': None}  # Dictionary to store color sensor readings
# Constants for direction control
CLOCKWISE = 1
ANTICLOCKWISE = -1
direction = 0
# Steering angles and bending values
steeringAngle = 1
predefinedSteeringValueBend = -17
predefinedReverseSteeringValueBend = -13.5
STOP = False  # Flag to indicate if the system should stop
steeringAngleForAfterSteering = 1.35  # Additional steering angle adjustment after steering
# Constants for frame dimensions and parking encoder value
FRAME_WIDTH = 1280
FRAME_HEIGHT = 960
parkingEncoderValue = -1925

# State variables
sleepTimeForMainCycle = 0.01  # Time to sleep in the main cycle to control loop speed
farFromLateralWall = True  # Flag to indicate if the system is far from the lateral wall
