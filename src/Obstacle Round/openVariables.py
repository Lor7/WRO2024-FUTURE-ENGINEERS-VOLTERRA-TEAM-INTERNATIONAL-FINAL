from numpy import array, uint8, pi
from colorama import Fore, Style
from colorSensor import ColorSensor
from gpiozero import Motor, AngularServo
from threading import Thread
from time import sleep

colorThread, defineColorThread = None, None
imageProcess = None
colorSensor = None

def initialize_color_sensor():
    global colorSensor
    while colorSensor is None:
        try:
            colorSensor = ColorSensor()
        except Exception as e:
            sleep(0.02)

color_sensor_thread = Thread(target=initialize_color_sensor)
color_sensor_thread.start()
color_sensor_thread.join()

STEERINGANGLEMID = 0
SERVOOFFSET = +50

motor = Motor(forward=24, backward=23, enable=18, pwm=True, pin_factory=None)
servo = AngularServo(pin=17, initial_angle=0, min_angle=-90, max_angle=90,
                     min_pulse_width=0.0006, max_pulse_width=0.0024,
                     pin_factory=None)

# Global Variables
lap, side, updatedSide = 0, 0, 0
pos, angle, speed, motorValue = None, None, 0.4, 0.2
colorDict = {'red': None, 'green': None, 'blue': None}
CLOCKWISE, ANTICLOCKWISE, direction = 1, -1, 0
steeringAngle, predefinedSteeringValueBend, predefinedReverseSteeringValueBend = 1, -17, -13.5
STOP = False
steeringAngleForAfterSteering = 1.35

# Constants
FRAME_WIDTH, FRAME_HEIGHT = 1280, 960
parkingEncoderValue = -1925

# State variables
sleepTimeForMainCycle = 0.015
farFromLateralWall = True
pauseTimeColorSensorRoundabout = 4.5
extraPauseTimeColorSensorRoundabout = 4.5
