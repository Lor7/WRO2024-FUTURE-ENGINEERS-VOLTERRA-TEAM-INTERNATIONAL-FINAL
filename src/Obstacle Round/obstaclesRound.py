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
from obstacleAlgorithm import avoidObstacle, setDirection as obstacleSetDirection, setState as obstacleSetState, setRememberObstacle, turnObstacleList, set_firstSideList_onFirstSide  # Import obstacle avoidance functions
from actuators import setSteeringAngle, reverse, setMinSteeringAngle, setDirection as actuatorsSetDirection, setState as actuatorsSetState, U_turn  # Import actuator functions
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

# Initialize roundabout-related variables
roundAbout = [0, 0, -5]  # Initial values for roundabout states
turnObstacleList(roundAbout)  # Update the obstacle list with roundabout values
obstacleMemoryStuck = [-1, -1, -1, -1]  # Initialize obstacle memory
obstaclesFirstSide = [0, 0]  # Initialize first side obstacles
carOnTheFirstSide = [False]  # Initialize car position on the first side
set_firstSideList_onFirstSide(obstaclesFirstSide, carOnTheFirstSide)  # Set initial state for first side list

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
    global directionInverted, carOnTheFirstSide, obstaclesFirstSide, imageProcessing
    
    
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
                    
                    # Handle obstacle recording for the first lap
                    if lap == 1 and updatedSide == 0:
                        setRememberObstacle(True)
                        print("Set remember-obstacle TRUE")
                        print("START recording the list of obstacles of the first side")
                        carOnTheFirstSide[0] = True
                    if lap == 1 and updatedSide == 1:
                        print("STOP recording the list of obstacles of the first side")
                        carOnTheFirstSide[0] = False
                        print(f"Obstacles first side: {obstaclesFirstSide}")
                    
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
    # Global variables used in the function
    global lap, side, updatedSide, state, timeLastLine, STUCK, obstacleMemoryStuck
    global direction, STOP, imageProcessing, imageProcessing_process
    global colorThread, defineColorThread
    # Initialize threads for color sensor data reading and defining color
    colorThread = Thread(target=colorSensor.readDataContinuously)
    defineColorThread = Thread(target=defineColor)
    
    # Variables for managing parking and obstacle handling
    farFromLateralWall, lastTimeObstacleWasSeen, flagBufferTimeToOvercome = True, (0, 0, 0), True
    estimatedDistance, = -1

    
    # Lock and shared values for inter-thread communication
    lock = Lock()
    shared_values, shared_values_copy = [i for i in range(32)], []
    imageProcessing = ImageProcessing(shared_values, lock, True, False, False)
    imageProcessing.run()
    
    # Start the color sensor thread
    colorThread.start()
    sleep(5)  # Give some time for threads to start
    
    # Setup complete
    print("Setup completed!")
    setSteeringAngle(2)  # Set an initial steering angle
    sleep(0.5)
    
    # Record starting time
    startingTime = time()
    
    # Initial motor value for movement
    motorValue = 0.235
    
    # Start the color defining thread
    defineColorThread.start()
    
    # Start the loop
    print("Loop started!")
    motor.forward(motorValue)  # Start moving forward
    
    # Initialize timing variables for parking and color detection
    chronograph = time()
    while True:
        try:
            # Move the motor forward at a set value
            motor.forward(motorValue)
            
            # Acquire the lock to access shared values
            lock.acquire()
            if shared_values[31] != 0:
                shared_values_copy = deepcopy(shared_values)  # Make a copy of the shared values
                shared_values[31] = 0  # Reset the status in shared values
            lock.release()

            
            # Handle obstacle detection
            if shared_values_copy[31] == PILLAR_WAS_SEEN:
                lastTimeObstacleWasSeen = (time(), shared_values_copy[4],
                                            shared_values_copy[0] if shared_values_copy[4] == GREENBLOCKID else shared_values_copy[0] + shared_values_copy[2],
                                            shared_values_copy[0], shared_values_copy[2], shared_values_copy[22] + shared_values_copy[24],
                                            shared_values_copy[1] + shared_values_copy[3], shared_values_copy[3])
                if shared_values_copy[1] + shared_values_copy[3] > 240:
                    obstacleMemoryStuck = (time(), shared_values_copy[4])
                
                state.add(OBSTACLE)
                state.discard(WALL)
                
                farFromLateralWall = avoidObstacle((shared_values_copy[0], shared_values_copy[1], shared_values_copy[2], shared_values_copy[3], shared_values_copy[4]),
                                (shared_values_copy[5], shared_values_copy[6], shared_values_copy[7], shared_values_copy[8]),
                                shared_values_copy[9], (shared_values_copy[10], shared_values_copy[11], shared_values_copy[12], shared_values_copy[13]),
                                (shared_values_copy[14], shared_values_copy[15], shared_values_copy[16], shared_values_copy[17], shared_values_copy[18]), 
                                shared_values_copy[19], shared_values_copy[20], shared_values_copy[21],
                                (shared_values_copy[22], shared_values_copy[23], shared_values_copy[24], shared_values_copy[25]))
                shared_values_copy[31] = 0

            # Handle wall detection
            elif shared_values_copy[31] == WALL_WAS_SEEN:
                state.add(WALL)
                state.discard(OBSTACLE)
                
                farFromLateralWall = makeDecision(shared_values_copy[0], shared_values_copy[1], shared_values_copy[2], shared_values_copy[3],
                            shared_values_copy[4], shared_values_copy[5], shared_values_copy[6],
                            (shared_values_copy[7], shared_values_copy[8], shared_values_copy[9], shared_values_copy[10]),
                            (shared_values_copy[11], shared_values_copy[12], shared_values_copy[13], shared_values_copy[14]),
                            (shared_values_copy[15], shared_values_copy[16], shared_values_copy[17], shared_values_copy[18]),
                            (shared_values_copy[19], shared_values_copy[20], shared_values_copy[21], shared_values_copy[22]),
                            shared_values_copy[23], shared_values_copy[24], shared_values_copy[25],
                            (shared_values_copy[26], shared_values_copy[27], shared_values_copy[28], shared_values_copy[29]),
                            (lastTimeObstacleWasSeen[0:2])
                            )
                shared_values_copy[31] = 0

            # Main cycle sleep
            sleep(sleepTimeForMainCycle)

        # Exception handling
        except KeyboardInterrupt:
            # Handle keyboard interrupt gracefully
            imageProcessing.stop()
            print("Keyboard interrupt!")
            motor.stop()
            setSteeringAngle(2)
            motor.stop()
            colorSensor.stop = True
            break
        except Exception as e:
            # Handle other exceptions
            print(f"Main cycle error: {e}, {traceback.format_exc()}")


if __name__ == '__main__':
    while True:
        try:
            loop()
            for i in range(0, 25):
                motor.stop()
                sleep(0.05)
            exit()
        except Exception as e:
            print("Initial try-catch", e)