# Import necessary modules
from threading import Thread, Lock
from copy import deepcopy
import traceback
from math import atan2
from time import time, sleep, perf_counter

from IMU import Imu  # Import IMU class
from openVariables import *  # Import open variables
from obstacleRecognitionConstants import *  # Import obstacle recognition constants
from finalImageProcessing import ImageProcessing  # Import image processing class
from brain import makeDecision, setDirection as brainSetDirection, setState as brainSetState  # Import brain functions
from obstacleAlgorithm import avoidObstacle, setDirection as obstacleSetDirection, setState as obstacleSetState, setRememberObstacle, turnObstacleList  # Import obstacle avoidance functions
from actuators import setSteeringAngle, setMinSteeringAngle, setDirection as actuatorsSetDirection, setState as actuatorsSetState, U_turn  # Import actuator functions
import sys

# Optional priority setting for the process (commented out)
"""
try:
    import os, psutil
    os_used = sys.platform
    process = psutil.Process(os.getpid())  
    if os_used == "win32":
        process.nice(psutil.HIGH_PRIORITY_CLASS)
    elif os_used == "linux":
        process.nice(psutil.IOPRIO_CLASS_RT)
except:
    pass
"""

# Constants for the parking and obstacle avoidance behavior
pauseTimeColorSensor = 6  # Time before starting (August) was 7.5
from differentStates import *  # Import different states

# Initialize states and other variables
state = set()  # Initialize an empty set for states
timeLastLine = [0]  # Initialize list to keep track of the last time line
brainSetState(state, timeLastLine)  # Set the state in the brain module
obstacleSetState(state, timeLastLine)  # Set the state in the obstacle module
actuatorsSetState(state, timeLastLine)  # Set the state in the actuators module

def setDirection():
    """
    Update the direction of the vehicle across various modules.
    """
    global direction
    obstacleSetDirection(direction)  # Set direction in the obstacle module
    brainSetDirection(direction)  # Set direction in the brain module
    imageProcessing.setDirection(direction)  # Set direction in the image processing module
    actuatorsSetDirection(direction)  # Set direction in the actuators module


def sideCounter():
    """
    Track and update the side of the vehicle based on sensor data.
    """
    global STOP, direction, side, updatedSide, CLOCKWISE, SIMPLE_REVERSE, ANTICLOCKWISE, lap, REVERSE, state, WIDE_CURVE, STUCK, pauseTimeColorSensor, timeLastLine
    
    
    while not STOP:
        try:
            while not STOP:
                # Wait for conditions to be clear before proceeding
                while REVERSE in state or WIDE_CURVE in state:
                    sleep(0.1)
                
                # Handle stuck state
                if STUCK in state:
                    sleep(4)
                    state.discard(STUCK)
                
                # Handle simple reverse state
                if SIMPLE_REVERSE in state:
                    sleep(2.5)
                    state.discard(SIMPLE_REVERSE)
                
                colorDict = colorSensor.datas  # Get color sensor data
                
                # Update side and lap based on color sensor readings
                if ((95 <= colorDict['red'] and colorDict['green'] <= 85 and colorDict['blue'] <= 85) or
                    (colorDict['red'] <= 82 and colorDict['green'] <= 105 and 103 <= colorDict['blue'])):
                    state.add(LINE)
                    if updatedSide == 3:
                        updatedSide = 0
                        lap += 1
                        imageProcessing.lap = lap
                    else:
                        updatedSide += 1
                    
                    # Update side and time for the last line
                    print(f"Lap: {lap}, Updated side: {updatedSide}\n")
                    timeLastLine[0] = time()
                    colorSensor.standby()  # Put color sensor in standby mode
                    sleep(pauseTimeColorSensor)  # Wait before resuming
                    colorSensor.resume()  # Resume color sensor operation
                    side = updatedSide
                    if lap == 4:
                        break
                sleep(0.0035)  # Short sleep between sensor checks
        except Exception as e:
            print(f"Side counter error: {e}")

    colorSensor.stop = True  # Stop the color sensor

def defineColor():
    """
    Determine the direction of the vehicle based on color sensor readings.
    """
    global direction, steeringAngleForAfterSteering, side, updatedSide, lap, imageProcessing, state, timeLastLine, colorSensor
    clockwiseCount, anticlockwiseCount = 0, 0  # Counters for direction determination
    
    while direction == 0:
        try:
            while direction == 0:
                colorDict = colorSensor.datas  # Get color sensor data
                
                # Determine direction based on color sensor readings
                if (95 <= colorDict['red'] and colorDict['green'] <= 85 and colorDict['blue'] <= 85) and \
                   (colorDict['green'] != 76.5 or colorDict['blue'] != 76.5):  # Clockwise
                    clockwiseCount += 1
                    anticlockwiseCount = 0
                    if clockwiseCount == 2:
                        direction = CLOCKWISE
                        state.add(LINE)
                        setDirection()
                        print(f"Direction defined clockwise, values: {colorDict}")
                elif (colorDict['red'] <= 82 and colorDict['green'] <= 105 and 103 <= colorDict['blue']):  # Anticlockwise
                    clockwiseCount = 0
                    anticlockwiseCount += 1
                    if anticlockwiseCount == 2:
                        direction = ANTICLOCKWISE
                        steeringAngleForAfterSteering = 3
                        state.add(LINE)
                        setDirection()
                        print(f"Direction defined anticlockwise, values: {colorDict}")
                sleep(0.0035)  # Short sleep between sensor checks
        except Exception as e:
            print(f"Define color error: {e}")
    
    timeLastLine[0] = time()  # Update time of the last line
    updatedSide += 1
    sideCounterThread = Thread(target=sideCounter)  # Start side counter thread
    colorSensor.standby()  # Put color sensor in standby mode
    sleep(pauseTimeColorSensor)  # Wait before resuming
    colorSensor.resume()  # Resume color sensor operation
    side = updatedSide
    sideCounterThread.start()  # Start side counter thread

def loop():
    """
    Main loop function that sets up threads, initializes parameters, and starts the robot's operations.
    """
    pass


if __name__ == '__main__':
    loop()