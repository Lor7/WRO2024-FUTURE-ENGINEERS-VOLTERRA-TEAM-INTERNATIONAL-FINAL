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
from brain import makeDecision, setDirection as brainSetDirection, setState as brainSetState, setReverseTime as brainSetReverseTime  # Import brain functions
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
flagSwitchDirection = True  # Flag to control direction switching
isRequiredExtraTimeRoundAbout = False  # Flag for extra time required at roundabouts
screenShot = True  # Flag to enable screenshots
flagStartedWaitedSecondsForRoundAbout, flagWaitedSecondsForRoundAbout = False, False  # Flags for roundabout timing
ignoreObstacleInTheMiddleOfTheFirstLane = False  # Flag to ignore obstacles in the first lane
flagMovementTowardLeft = True  # Flag for movement direction

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

# Initialize additional flags and variables
roundAboutBufferTimeExpired = False
directionInverted = False

def setDirection():
    """
    Update the direction of the vehicle across various modules.
    """
    global direction
    obstacleSetDirection(direction)  # Set direction in the obstacle module
    brainSetDirection(direction)  # Set direction in the brain module
    imageProcessing.setDirection(direction)  # Set direction in the image processing module
    actuatorsSetDirection(direction)  # Set direction in the actuators module

def waitSecondsForRoundAbout(seconds):
    """
    Wait for a specified number of seconds for roundabout processing.
    """
    global flagWaitedSecondsForRoundAbout
    print(f"WAITING {seconds} for roundabout")  # Log wait start
    sleep(seconds)  # Wait for the specified time
    print(f"WAITED {seconds} for roundabout")  # Log wait end
    flagWaitedSecondsForRoundAbout = True  # Set flag to indicate completion

def takeScreenshot():
    """
    Take a screenshot based on specific conditions related to the roundabout.
    """
    global screenShot, flagSwitchDirection, roundAbout, isRequiredExtraTimeRoundAbout
    flagSwitchDirection = False  # Disable direction switching
    screenShot = False  # Disable screenshot flag
    if ((direction == ANTICLOCKWISE and roundAbout[0] == REDBLOCKID) or
        (direction == CLOCKWISE and roundAbout[0] == GREENBLOCKID)) and time() - roundAbout[1] < 4:
        # Conditional screenshot timing based on direction and roundabout ID
        if False and direction == ANTICLOCKWISE and roundAbout[0] == REDBLOCKID and time() - roundAbout[1] < 4.5:
            print("Waiting 2 seconds to take a screenshot")  # Log wait time
            sleep(2)  # Wait for 2 seconds
        else:
            print("Waiting 1.5 seconds to take a screenshot")  # Log wait time
            sleep(1.5)  # Wait for 1.5 seconds
    elif ((direction == CLOCKWISE and roundAbout[0] == REDBLOCKID) or
          (direction == ANTICLOCKWISE and roundAbout[0] == GREENBLOCKID)) and time() - roundAbout[1] < 3:
        # Conditional screenshot timing based on direction and roundabout ID
        if isRequiredExtraTimeRoundAbout:
            print("Waiting 1.5 seconds to take a screenshot")  # Log wait time
            sleep(1.5)  # Wait for 1.5 seconds
        else:
            print("Waiting 0.6 seconds to take a screenshot")  # Log wait time
            sleep(0.6)  # Wait for 0.6 seconds
    else:
        pass
    flagSwitchDirection = True  # Re-enable direction switching
    print("Time waited to take a screenshot")  # Log screenshot completion

def bufferTimeOvercomeObstacle():
    """
    Implement buffer time to overcome obstacles based on the roundabout state.
    """
    global flagSwitchDirection, roundAbout
    # Determine wait time based on roundabout state
    if time() - roundAbout[1] < 1:
        timeToWait = 2
    else:
        timeToWait = 1
    print(f"Using bufferTimeOvercomeObstacle, time to wait: {timeToWait}")  # Log wait time
    sleep(timeToWait)  # Wait for the determined time
    flagSwitchDirection = True  # Re-enable direction switching
    
def bufferRoundAboutTime():
    """
    Manage buffer time at a roundabout based on direction and conditions.
    """
    global roundAboutBufferTimeExpired, direction, isRequiredExtraTimeRoundAbout
    timeToSleep = 4  # Default sleep time
    
    try:
        # Adjust time based on whether extra time is required
        if isRequiredExtraTimeRoundAbout:
            print("Extra time conceded and obtained")
            if direction == ANTICLOCKWISE:
                timeToSleep = 4.5  # Increase time for anticlockwise direction
                # Additional time if specific roundabout conditions are met
                if roundAbout[0] == REDBLOCKID and time() - roundAbout[1] < 4.5:
                    timeToSleep += 1
                elif roundAbout[0] == GREENBLOCKID:
                    timeToSleep -= 0.5
                print(f"Time to sleep increased to: {timeToSleep}")
            if direction == CLOCKWISE and roundAbout[0] == GREENBLOCKID:
                timeToSleep += 0.25
                print(f"Time to sleep increased to: {timeToSleep}")
        else:
            # Adjust time if extra time is not needed
            if (direction == ANTICLOCKWISE and (roundAbout[0] == GREENBLOCKID or 
                (roundAbout[0] == REDBLOCKID and time() - roundAbout[1] > 4))) or \
               (direction == CLOCKWISE and (roundAbout[0] == REDBLOCKID or 
                (roundAbout[0] == GREENBLOCKID and time() - roundAbout[1] > 4))):
                timeToSleep -= 1.5
            elif (direction == ANTICLOCKWISE and roundAbout[0] == REDBLOCKID) or \
                 (direction == CLOCKWISE and roundAbout[0] == GREENBLOCKID):
                timeToSleep -= 0.75
            print(f"There is no need for extra time! (So ultrasound said), time to sleep {timeToSleep}")
    except Exception as e:
        print(f"Buffer roundabout time error: {e}")
    
    sleep(timeToSleep)  # Sleep for the calculated buffer time
    setRememberObstacle(False)  # Reset obstacle memory
    roundAboutBufferTimeExpired = True  # Indicate that buffer time has expired
    print("Set remember-obstacle FALSE")

def sideCounter():
    """
    Track and update the side of the vehicle based on sensor data.
    """
    global STOP, direction, side, updatedSide, CLOCKWISE, SIMPLE_REVERSE, ANTICLOCKWISE, lap, REVERSE, state, WIDE_CURVE, STUCK, pauseTimeColorSensor, timeLastLine, extraPauseTimeColorSensorRoundabout
    global directionInverted, carOnTheFirstSide, obstaclesFirstSide, imageProcessing
    
    flagExtraRoundAboutSleep = True  # Flag to manage extra sleep
    
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

def handleObstacleInTheMiddleOfTheLane(shared_values):
    """
    Determine whether to ignore an obstacle in the middle of the lane based on sensor data.
    """
    global isRequiredExtraTimeRoundAbout, ignoreObstacleInTheMiddleOfTheFirstLane
    
    try:
        if not isRequiredExtraTimeRoundAbout:
            trueIndex, maxIndex, index, votes = 0, 13, 0, 0
            while index < 9 and trueIndex < maxIndex:
                if shared_values[31] == WALL_WAS_SEEN:
                    votes += 1
                else:
                    # Check if the distance indicates an obstacle is present
                    shared_values_copy = deepcopy(shared_values)
                    if 10 * 633.3333333333334 / 2 / shared_values[3] < 60:
                        votes += 1
                    else:
                        votes -= 1
                    index += 1
                sleep(0.1)
                trueIndex += 1
            # Decide whether to ignore the obstacle based on votes
            ignoreObstacleInTheMiddleOfTheFirstLane = votes > 0
            print(f"Votes for ignoring obstacle in the middle of the lane: {votes}, ignore: {ignoreObstacleInTheMiddleOfTheFirstLane}")
    except Exception as e:
        print(f"Error handle obstacle in the middle of the lane: {e}")

def handleExtraTimeForRoundAbout():
    """
    Determine if extra time is needed for the roundabout based on distance measurements.
    """
    try:
        global isRequiredExtraTimeRoundAbout
        
        from gpiozero import DistanceSensor
        # Initialize distance sensor
        distanceSensor = DistanceSensor(echo=10, trigger=9, queue_len=1,
                                        max_distance=2.5, partial=False,
                                        threshold_distance=0.1, pin_factory=Device.pin_factory)
        
        trueIndex, index, flagExtraTimeNeeded = 0, 0, 0
        while index < 9 and trueIndex < 50:
            distance = distanceSensor.distance
            if not (distance <= 0.1 or distance >= 2.0):
                # Determine if extra time is needed based on distance
                if distance < 0.9 or distance > 1.45:
                    flagExtraTimeNeeded -= 1
                elif 0.9 < distance < 1.4:
                    flagExtraTimeNeeded += 1
                index += 1
                sleep(0.025)
            trueIndex += 1
        
        # Clean up distance sensor
        try:
            del distanceSensor
        except Exception as e:
            print(e)
        
        # Set the flag based on the votes
        if flagExtraTimeNeeded > 0:
            print(f"Extra time needed, votes: {flagExtraTimeNeeded}")
            isRequiredExtraTimeRoundAbout = True
        elif flagExtraTimeNeeded < 0:
            print(f"No need for extra time, votes: {flagExtraTimeNeeded}")
            isRequiredExtraTimeRoundAbout = False
        else:
            print(f"No need for extra time (IMPOSSIBLE TO UNDERSTAND), votes: {flagExtraTimeNeeded}")
            isRequiredExtraTimeRoundAbout = False
    except:
        pass
    
def loop():
    """
    Main loop function that sets up threads, initializes parameters, and starts the robot's operations.
    """
    # Global variables used in the function
    global lap, side, updatedSide, state, timeLastLine, STUCK, obstacleMemoryStuck
    global direction, STOP, imageProcessing, imageProcessing_process
    global colorThread, defineColorThread
    global extraPauseTimeColorSensorRoundabout, roundAboutBufferTimeExpired, directionInverted, flagSwitchDirection, screenShot
    global flagStartedWaitedSecondsForRoundAbout, flagMovementTowardLeft, flagWaitedSecondsForRoundAbout, stopChecking

    # Initialize threads for color sensor data reading and defining color
    colorThread = Thread(target=colorSensor.readDataContinuously)
    defineColorThread = Thread(target=defineColor)
    
    # Variables for managing parking and obstacle handling
    farFromLateralWall, lastTimeObstacleWasSeen, flagBufferTimeToOvercome = True, (0, 0, 0), True
    estimatedDistance, thresholdDistance, performTimedMovement, roundAboutCalculationNotDone = -1, 0, True, True
    obstaclePreviousData, wallBeingViewedTime = [], time()
    
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
    
    # Handle extra time for roundabout and obstacles
    handleExtraTimeForRoundAbout()
    handleObstacleInTheMiddleOfTheLane(shared_values)    
    
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
                wallBeingViewedTime = time()
                
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
            
            
            # Check if the conditions for detecting a stuck situation are met.
            # Note: The condition `if False` is used here to disable this block of code.
            if False and imu.noAngularVelocitiesCount > 10 and time() - chronograph > 4:
                # If the prototype is detected as stuck
                print("Prototype stuck!")
                state.add(STUCK)  # Add the STUCK state to the set of active states

                # Check if the obstacle that caused the robot to be stuck is a red block
                # and if the time since the obstacle was detected is less than 3.5 seconds
                if obstacleMemoryStuck[1] == REDBLOCKID and time() - obstacleMemoryStuck[0] < 3.5:
                    print("Reverse case 1")
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Set the steering angle to 2 degrees
                    sleep(0.4)  # Wait for 0.4 seconds
                    motor.backward(motorValue)  # Move the motor backward
                    sleep(0.7)  # Wait for 0.7 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(-12)  # Adjust steering angle to -12 degrees
                    motor.backward(motorValue)  # Move the motor backward
                    sleep(0.9)  # Wait for 0.9 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(10)  # Adjust steering angle to 10 degrees
                    motor.forward(motorValue)  # Move the motor forward
                    sleep(0.5)  # Wait for 0.5 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Reset steering angle to 2 degrees

                # Check if the obstacle that caused the robot to be stuck is a red block
                # and if the time since the obstacle was detected is less than 3.5 seconds
                elif obstacleMemoryStuck[1] == REDBLOCKID and time() - obstacleMemoryStuck[0] < 3.5:
                    print("Reverse case 2")
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Set the steering angle to 2 degrees
                    sleep(0.4)  # Wait for 0.4 seconds
                    motor.backward(motorValue)  # Move the motor backward
                    sleep(0.7)  # Wait for 0.7 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(12)  # Adjust steering angle to 12 degrees
                    motor.backward(motorValue)  # Move the motor backward
                    sleep(0.9)  # Wait for 0.9 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(-10)  # Adjust steering angle to -10 degrees
                    motor.forward(motorValue)  # Move the motor forward
                    sleep(0.5)  # Wait for 0.5 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Reset steering angle to 2 degrees

                # Check if the obstacle that caused the robot to be stuck is a green block
                # and if the time since the obstacle was detected is less than 3.5 seconds
                # and if the direction is anticlockwise
                elif obstacleMemoryStuck[1] == GREENBLOCKID and time() - lastTimeObstacleWasSeen[0] < 3.5 and direction == ANTICLOCKWISE:
                    print("Reverse case green")
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Set the steering angle to 2 degrees
                    sleep(0.4)  # Wait for 0.4 seconds
                    motor.backward(motorValue)  # Move the motor backward
                    sleep(0.7)  # Wait for 0.7 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(12)  # Adjust steering angle to 12 degrees
                    motor.backward(motorValue)  # Move the motor backward
                    sleep(0.9)  # Wait for 0.9 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(-10)  # Adjust steering angle to -10 degrees
                    motor.forward(motorValue)  # Move the motor forward
                    sleep(0.5)  # Wait for 0.5 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Reset steering angle to 2 degrees

                # Handle the case where the robot is stuck, but none of the specific conditions above are met
                else:
                    print("Reverse case 3")
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Set the steering angle to 2 degrees
                    sleep(0.4)  # Wait for 0.4 seconds
                    motor.backward(motorValue)  # Move the motor backward
                    for i in range(6):  # Repeat the following actions 6 times
                        setSteeringAngle(2)  # Set the steering angle to 2 degrees
                        sleep(0.3)  # Wait for 0.3 seconds
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Reset steering angle to 2 degrees

                # Uncomment the following line to discard the STUCK state if needed
                # state.discard(STUCK)  # Remove the STUCK state from the set of active states

            # Check specific conditions for lap and updatedSide
            if lap == 2 and updatedSide == 0 and (flagSwitchDirection or screenShot):
                    
                #print(f"Round about: {roundAbout}")
                # Check if a screenshot should be taken
                if screenShot:
                    # Disable obstacle memory
                    setRememberObstacle(False)
                    # Start a new thread to take a screenshot
                    threadTakeScreenshot = Thread(target = takeScreenshot)
                    threadTakeScreenshot.start()
                    print("Set remember-obstacle FALSE")
                    continue  # Skip the rest of the loop and start the next iteration

                # Check if roundabout calculation has not been done yet
                if roundAboutCalculationNotDone:
                    roundAboutCalculationNotDone = False  # Mark the calculation as done

                    # Check the type of obstacle and update roundAbout based on it
                    if obstaclesFirstSide[1] != 0:
                        if obstaclesFirstSide[0] == REDBLOCKID:
                            performTimedMovement = False
                            roundAbout[0] = REDBLOCKID
                            roundAbout[1] = time() + 1  # Set roundabout timestamp
                            print("Roundabout locked red (red-...)")
                        else:
                            performTimedMovement = False
                            roundAbout[0] = GREENBLOCKID
                            roundAbout[1] = time() + 1  # Set roundabout timestamp
                            print("Roundabout locked green (green-...)")

                    # Check if the obstacle in the middle of the first lane should be ignored or not
                    elif obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == GREENBLOCKID and not(ignoreObstacleInTheMiddleOfTheFirstLane) and isRequiredExtraTimeRoundAbout:
                        print("Roundabout locked green by (camera+ultrasonic)")
                        roundAbout[0] = GREENBLOCKID
                        roundAbout[1] = time() + 2  # Set roundabout timestamp with extra time
                        performTimedMovement = False

                    elif obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == REDBLOCKID and not(ignoreObstacleInTheMiddleOfTheFirstLane) and isRequiredExtraTimeRoundAbout:
                        print("Roundabout locked red by (camera+ultrasonic)")
                        roundAbout[0] = REDBLOCKID
                        roundAbout[1] = time() + 2  # Set roundabout timestamp with extra time
                        performTimedMovement = False

                    # Handle cases based on the last seen obstacle and its distance
                    elif (roundAbout[0] != lastTimeObstacleWasSeen[1]) or (time() - lastTimeObstacleWasSeen[0] < 1.5 and not(lastTimeObstacleWasSeen[-2] > 350 or
                        (lastTimeObstacleWasSeen[2] > 480 and lastTimeObstacleWasSeen[1] == GREENBLOCKID) or (lastTimeObstacleWasSeen[2] < 160 and lastTimeObstacleWasSeen[1] == REDBLOCKID))):
                        
                        estimatedDistance = 10 * 633.3333333333334 / 2 / lastTimeObstacleWasSeen[-1]
                        thresholdDistance = 100

                        # Update thresholdDistance based on the direction and type of obstacle
                        if (time() - roundAbout[1] > 4 and not((direction == ANTICLOCKWISE and roundAbout[0] == REDBLOCKID) or (direction == CLOCKWISE and roundAbout[0] == GREENBLOCKID))) or (time() - roundAbout[1] > 5):
                            pass
                        elif (roundAbout[0] == REDBLOCKID and direction == ANTICLOCKWISE) or (roundAbout[0] == GREENBLOCKID and direction == CLOCKWISE):
                            thresholdDistance = 110
                            print(f"thresholdDistance updated to {thresholdDistance}")
                        elif (roundAbout[0] == REDBLOCKID and direction == CLOCKWISE) or (roundAbout[0] == GREENBLOCKID and direction == ANTICLOCKWISE):
                            thresholdDistance = 90
                            print(f"thresholdDistance updated to {thresholdDistance}")
                        
                        # Add extra time if required
                        if isRequiredExtraTimeRoundAbout:
                            thresholdDistance += 40
                        
                        performTimedMovement = True

                        # Decide whether to perform timed movement based on estimated distance
                        if estimatedDistance > thresholdDistance:
                            print(f"estimatedDistance > thresholdDistance, {estimatedDistance} > {thresholdDistance}")
                            if estimatedDistance > 110:
                                performTimedMovement = True
                            else:
                                performTimedMovement = False
                        else:
                            print(f"estimatedDistance < thresholdDistance, {estimatedDistance} < {thresholdDistance}")
                            if isRequiredExtraTimeRoundAbout and estimatedDistance < thresholdDistance:
                                if obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == GREENBLOCKID and ignoreObstacleInTheMiddleOfTheFirstLane:
                                    print("Round about NOT updated (alpha)")
                                else:
                                    roundAbout[0] = lastTimeObstacleWasSeen[1]  # Update roundabout id
                                    roundAbout[1] = time() + estimatedDistance / 12
                                    print("Round about updated (alpha)")
                                performTimedMovement = False
                            elif time() - roundAbout[1] > 4 and estimatedDistance < 70:
                                if obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == GREENBLOCKID and ignoreObstacleInTheMiddleOfTheFirstLane:
                                    print("Round about NOT updated (beta)")
                                else:
                                    roundAbout[0] = lastTimeObstacleWasSeen[1]  # Update roundabout id
                                    roundAbout[1] = time() + estimatedDistance / 12
                                    print("Round about updated (beta)")
                                performTimedMovement = False
                            elif roundAbout[0] == REDBLOCKID and estimatedDistance < 70:
                                if obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == GREENBLOCKID and ignoreObstacleInTheMiddleOfTheFirstLane:
                                    print("Round about NOT updated (gamma)")
                                else:
                                    roundAbout[0] = lastTimeObstacleWasSeen[1]  # Update roundabout id
                                    roundAbout[1] = time() + estimatedDistance / 12
                                    print("Round about updated (gamma)")
                                performTimedMovement = False
                            elif roundAbout[0] == GREENBLOCKID and estimatedDistance < 70:
                                if obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == GREENBLOCKID and ignoreObstacleInTheMiddleOfTheFirstLane:
                                    print("Round about NOT updated (delta)")
                                else:
                                    roundAbout[0] = lastTimeObstacleWasSeen[1]  # Update roundabout id
                                    roundAbout[1] = time() + estimatedDistance / 12
                                print("Round about updated (delta)")
                                performTimedMovement = False
                            else:
                                if estimatedDistance > 110:
                                    performTimedMovement = True
                                else:
                                    performTimedMovement = False

                        performTimedMovement = False
                        if performTimedMovement:
                            print("performTimedMovement TRUE")
                        else:
                            print("performTimedMovement FALSE")
                    
                    # Handle timed movement if required
                    if performTimedMovement and roundAbout[0] == REDBLOCKID:
                        if ((estimatedDistance == -1 or estimatedDistance > thresholdDistance) and not(flagStartedWaitedSecondsForRoundAbout)):
                            flagStartedWaitedSecondsForRoundAbout = True
                            coroutineWaitSecondsForRoundAbout = Thread(target = waitSecondsForRoundAbout, args = (3,))
                            coroutineWaitSecondsForRoundAbout.start()
                            continue
                        elif not(flagWaitedSecondsForRoundAbout):
                            if time() - timeLastLine[0] > 1.5 and time() - lastTimeObstacleWasSeen[0] < 0.75 and not(lastTimeObstacleWasSeen[-2] > 340 or
                                (lastTimeObstacleWasSeen[2] > 480 and lastTimeObstacleWasSeen[1] == GREENBLOCKID) or (lastTimeObstacleWasSeen[2] < 160 and lastTimeObstacleWasSeen[1] == REDBLOCKID)):
                                estimatedDistance = 10 * 633.3333333333334 / 2 / lastTimeObstacleWasSeen[-1]
                                if estimatedDistance < 27:
                                    print("performTimedMovement distanza stimata < 27")
                                    pass
                                else:
                                    continue
                            else:
                                continue
                        elif flagWaitedSecondsForRoundAbout:
                            print("performTimedMovement coroutine di tempo conclusa")
                            pass
                        else:
                            continue

                    # Handle the case for REDBLOCKID if not started waiting yet
                    elif roundAbout[0] == REDBLOCKID:
                        if not(flagStartedWaitedSecondsForRoundAbout):
                            flagStartedWaitedSecondsForRoundAbout = True
                            coroutineWaitSecondsForRoundAbout = Thread(target = waitSecondsForRoundAbout, args = (8,))
                            coroutineWaitSecondsForRoundAbout.start()
                            continue
                        elif flagWaitedSecondsForRoundAbout:
                            print("DO NOT performTimedMovement, coroutine conclusa")
                            pass
                        else:
                            y_h = lastTimeObstacleWasSeen[-2]
                            if len(obstaclePreviousData) <= 1:
                                elapsedTime = 0
                                obstaclePreviousData = [y_h, lastTimeObstacleWasSeen[1], time()]
                            else:
                                if (abs(y_h - obstaclePreviousData[0]) > 50) or (obstaclePreviousData[1] != lastTimeObstacleWasSeen[1]) or (time() - lastTimeObstacleWasSeen[0] > 1.2) or (WALL in state and lastTimeObstacleWasSeen[0] - wallBeingViewedTime > 1.2):
                                    elapsedTime = time() - obstaclePreviousData[2]
                                else:
                                    obstaclePreviousData = [y_h, lastTimeObstacleWasSeen[1], time()]
                                    if lastTimeObstacleWasSeen[1] == REDBLOCKID:
                                        #print(f"Before overcoming redblockid")
                                        pass
                                    else:
                                        pass
                                        #print(f"Before overcoming greenblockid")
                            if elapsedTime > 2:  # Check if obstacle is considered overcome
                                print("Obstacle overcomed")
                                pass
                            else:
                                #print("Obstacle yet to overcome")
                                continue

                # Check if the current obstacle is REDBLOCKID
                if roundAbout[0] == REDBLOCKID:
                    # Update the state to indicate a wide curve maneuver is in progress
                    state.add(WIDE_CURVE)
                    
                    # Check the obstacles on the first side
                    if (obstaclesFirstSide[1] == REDBLOCKID and obstaclesFirstSide[0] == REDBLOCKID):
                        # If both obstacles are red, set the movement direction to left
                        flagMovementTowardLeft = True
                        print(f"Movimento di inversione di marcia verso sinistra #1, diff tempo: {time() - lastTimeObstacleWasSeen[0]}")
                    elif (obstaclesFirstSide[1] == GREENBLOCKID and obstaclesFirstSide[0] == REDBLOCKID):
                        # If the first obstacle is green and the second is red
                        if flagWaitedSecondsForRoundAbout:
                            # If time has been waited for the roundabout, move right
                            flagMovementTowardLeft = False
                            print(f"Movimento di inversione di marcia verso destra (e NON sinistra) #2, diff tempo: {time() - lastTimeObstacleWasSeen[0]}")
                        else:
                            # Otherwise, move left
                            flagMovementTowardLeft = True
                            print(f"Movimento di inversione di marcia verso sinistra #2, diff tempo: {time() - lastTimeObstacleWasSeen[0]}")
                    elif (obstaclesFirstSide[0] == GREENBLOCKID and not(ignoreObstacleInTheMiddleOfTheFirstLane)):
                        # If the first obstacle is green and not ignoring obstacles in the middle of the first lane, move right
                        flagMovementTowardLeft = False
                        print(f"Movimento di inversione di marcia verso destra #1")
                    elif (obstaclesFirstSide[1] == 0 and obstaclesFirstSide[0] == GREENBLOCKID):
                        # If there's no obstacle on the second side and the first is green
                        if ignoreObstacleInTheMiddleOfTheFirstLane:
                            # If ignoring obstacles in the middle, move left
                            print(f"Movimento di inversione di marcia verso sinistra (ignoreObstacleInTheMiddleOfTheFirstLane)")
                            flagMovementTowardLeft = True
                        else:
                            # Otherwise, move right
                            flagMovementTowardLeft = False
                            print(f"Movimento di inversione di marcia verso destra")
                    elif obstaclesFirstSide[0] == 0 and direction == CLOCKWISE:
                        # If there's no obstacle on the first side and the direction is clockwise, move left
                        flagMovementTowardLeft = True
                    elif obstaclesFirstSide[0] == 0 and direction == ANTICLOCKWISE:
                        # If there's no obstacle on the first side and the direction is anticlockwise, move right
                        flagMovementTowardLeft = False
                    
                    # Reset flag for switching direction and update direction
                    flagSwitchDirection = False
                    directionInverted = True
                    direction = -1 * direction
                    setDirection()
                    state.discard(WIDE_CURVE)  # Remove wide curve state
                    print(f"Direction updated: {direction}")

                    # Execute timed movement if required
                    performTimedMovement = False
                    if performTimedMovement:
                        # Stop the motor, set steering angle, move forward, and pause
                        motor.stop()
                        setSteeringAngle(2)
                        motor.forward(motorValue)
                        sleep(0.8)
                        motor.stop()
                        
                        # Perform a U-turn depending on the direction
                        if flagMovementTowardLeft:
                            if obstaclesFirstSide[1] == GREENBLOCKID:
                                # Check if stuck against a wall and perform a U-turn
                                checkIfStuckAgainstTheWallThread = Thread(target=checkIfStuckAgainstTheWall, args=(2.5,))
                                checkIfStuckAgainstTheWallThread.start()
                                print("U_TURN #1")
                                U_turn(flagMovementTowardLeft, seconds=2.5)
                            else:
                                checkIfStuckAgainstTheWallThread = Thread(target=checkIfStuckAgainstTheWall, args=(3.5,))
                                checkIfStuckAgainstTheWallThread.start()
                                print("U_TURN #2")
                                U_turn(flagMovementTowardLeft, seconds=3.5)
                        else:
                            checkIfStuckAgainstTheWallThread = Thread(target=checkIfStuckAgainstTheWall, args=(3.5,))
                            checkIfStuckAgainstTheWallThread.start()
                            print("U_TURN #3")
                            U_turn(flagMovementTowardLeft, seconds=3.5)
                    else:
                        # Implementation of movement after marking the obstacle
                        motor.stop()
                        setSteeringAngle(2)
                        motor.forward(motorValue)
                        sleep(0.8)
                        motor.stop()
                        
                        # Perform a U-turn depending on the direction
                        if flagMovementTowardLeft:
                            if obstaclesFirstSide[1] == GREENBLOCKID:
                                checkIfStuckAgainstTheWallThread = Thread(target=checkIfStuckAgainstTheWall, args=(3,))
                                checkIfStuckAgainstTheWallThread.start()
                                print("U_TURN #4")
                                U_turn(flagMovementTowardLeft, seconds=3)
                            else:
                                checkIfStuckAgainstTheWallThread = Thread(target=checkIfStuckAgainstTheWall, args=(3.5,))
                                checkIfStuckAgainstTheWallThread.start()
                                print("U_TURN #5")
                                U_turn(flagMovementTowardLeft, seconds=3.5)
                        else:
                            checkIfStuckAgainstTheWallThread = Thread(target=checkIfStuckAgainstTheWall, args=(3.5,))
                            checkIfStuckAgainstTheWallThread.start()
                            print("U_TURN #6")
                            U_turn(flagMovementTowardLeft, seconds=3.5)
                    
                    # Stop the movement and adjust steering if stuck while performing U-turn
                    stopChecking = True
                    if (direction == ANTICLOCKWISE and not flagMovementTowardLeft) or (direction == CLOCKWISE and flagMovementTowardLeft):
                        print("brainSetReverseTime, roundabout close to the center")
                        brainSetReverseTime([time()])
                        
                    # Final adjustments and reset
                    sleep(0.4)
                    state.discard(WIDE_CURVE)
                    setSteeringAngle(2, importance=3)
                    motor.forward(motorValue)
                    timeLastLine[0] = 0
                    print("Finished roundAbout")
                    roundAbout[2] = time()


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