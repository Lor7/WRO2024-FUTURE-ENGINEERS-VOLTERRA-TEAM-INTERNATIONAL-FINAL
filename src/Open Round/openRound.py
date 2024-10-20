from threading import Thread
from multiprocessing import Process, Lock
from copy import deepcopy
from time import time, sleep
from openVariables import *
from wallRecognitionConstants import *
from IMU import Imu
from finalImageProcessing import ImageProcessing
from brain import makeDecision, setDirection as brainSetDirection, setState as brainSetState
from actuators import setSteeringAngle, reverse
import sys

# Pause time for the color sensor to stabilize between readings
pauseTimeColorSensor = 7

from differentStates import *

# Initialize global variables
state = set()  # Set to keep track of current states
timeLastLine = [0]  # List to store the last time a line was detected
brainSetState(state, timeLastLine)  # Initialize the brain state with the current state
parkingTime = 5.0  # Time allocated for parking

# Initialize the IMU (Inertial Measurement Unit) sensor
imu = Imu()

def setDirection():
    """
    Update the direction in the brain and image processing modules.
    """
    global direction
    brainSetDirection(direction)  # Update the direction in the brain
    imageProcessing.setDirection(direction)  # Pass the direction to image processing

def sideCounter():
    """
    Monitor and update the side counter based on the color sensor data.
    """
    global STOP, direction, side, updatedSide, CLOCKWISE, ANTICLOCKWISE, lap, REVERSE, state, WIDE_CURVE, timeLastLine, STUCK
    while not STOP:
        try:
            while not STOP:
                while REVERSE in state or WIDE_CURVE in state:
                    sleep(0.1)  # Wait if in reverse or wide curve state
                if STUCK in state:
                    sleep(4.5)  # Delay if stuck
                    state.discard(STUCK)  # Remove the stuck state
                colorDict = colorSensor.datas  # Get color sensor data
                if ((95 <= colorDict['red'] and colorDict['green'] <= 85 and colorDict['blue'] <= 85) or
                    (colorDict['red'] <= 82 and colorDict['green'] <= 105 and 103 <= colorDict['blue'])):
                    # Check for line detection based on color sensor readings
                    state.add(LINE)  # Add line state
                    timeLastLine[0] = time()  # Update the last line detection time
                    if updatedSide == 3:
                        updatedSide = 0
                        lap += 1  # Increment lap count if side updated
                    else:
                        updatedSide += 1
                    print(f"Lap: {lap}, Updated side: {updatedSide}\n")
                    sleep(pauseTimeColorSensor)  # Pause after detecting the line
                    side = updatedSide
                    if lap == 3:
                        break  # Exit loop after completing 3 laps
                sleep(0.001)  # Short sleep to reduce CPU usage
        except Exception as e:
            print(str(e))  # Print any exceptions encountered

    colorSensor.stop = True  # Stop the color sensor when done

def defineColor():
    """
    Define the direction based on the color sensor data and set the corresponding state.
    """
    global direction, steeringAngleForAfterSteering, side, updatedSide, lap, imageProcessing, state, timeLastLine
    clockwiseCount, anticlockwiseCount = 0, 0
    while direction == 0:
        try:
            while direction == 0:
                colorDict = colorSensor.datas  # Get color sensor data
                if (95 <= colorDict['red'] and colorDict['green'] <= 85 and colorDict['blue'] <= 85) and (colorDict['green'] != 76.5 or colorDict['blue'] != 76.5):
                    # Check for clockwise direction
                    clockwiseCount += 1
                    anticlockwiseCount = 0
                    if clockwiseCount == 2:
                        direction = CLOCKWISE
                        state.add(LINE)  # Update state to line detected
                        setDirection()  # Set the direction
                        print(f"Direction defined clockwise, values: {colorDict}")
                elif ((colorDict['red'] <= 82 and colorDict['green'] <= 105 and 103 <= colorDict['blue'])):
                    # Check for anticlockwise direction
                    clockwiseCount = 0
                    anticlockwiseCount += 1
                    if anticlockwiseCount == 2:
                        direction = ANTICLOCKWISE
                        steeringAngleForAfterSteering = 3
                        state.add(LINE)  # Update state to line detected
                        setDirection()  # Set the direction
                        print(f"Direction defined anticlockwise, values: {colorDict}")
                sleep(0.0025)  # Short sleep to reduce CPU usage
        except Exception as e:
            #print(e)  # Suppress exceptions for now
            pass
    updatedSide += 1
    timeLastLine[0] = time()  # Update the time after defining color
    sideCounterThread = Thread(target=sideCounter)  # Start side counter thread
    sleep(pauseTimeColorSensor)  # Pause before starting side counter
    side = updatedSide
    sideCounterThread.start()  # Start the side counter thread

def handleLed_waitForButton():
    """
    Handle LED blinking and wait for button press to start the system.
    """
    from gpiozero import Button, LED
    button = Button(26)  # Initialize button on GPIO pin 26
    led = LED(12)  # Initialize LED on GPIO pin 12
    del Button, LED  # Clean up imports to avoid conflicts
    led.blink()  # Blink the LED
    print("Waiting for the button!")
    if button.is_pressed:
        button.wait_for_release()  # Wait for button release
    else:
        button.wait_for_press()  # Wait for button press
    sleep(2.5)  # Pause after button press
    led.off()  # Turn off the LED

def loop():
    """
    Main loop to manage the robot's operation, including sensor readings, state updates, and control actions.
    """
    global lap, side, updatedSide, state, parkingTime
    global direction, STOP, imageProcessing, imageProcessing_process
    global colorThread, defineColorThread
    
    # Initialize threads for color sensor and IMU readings
    colorThread = Thread(target=colorSensor.readDataContinuously)
    imuThread = Thread(target=imu.continuouslyReadData)
    defineColorThread = Thread(target=defineColor)
    
    farFromLateralWall, lastTimeFarFromLateralWall = True, 0
    flagStuckParking = False
    showFrameFlag = True
    
    # Check for command-line arguments to determine if frame display should be shown
    if len(sys.argv) > 1:
        if sys.argv[1] == "--dontShowFrameFlag":
            showFrameFlag = False
            print("dontShowFrameFlag")
    else:
        print("No command arguments were specified")
    
    # Initialize image processing with shared values and lock for thread-safe operations
    lock = Lock()
    shared_values, shared_values_copy = [i for i in range(31)], []
    imageProcessing = ImageProcessing(shared_values, lock, showFrameFlag, False)
    imageProcessing.run()  # Start the image processing

    colorThread.start()  # Start the color sensor thread
    imuThread.start()  # Start the IMU thread
    sleep(5)  # Delay for sensor initialization

    print("Setup completed!")
    setSteeringAngle(0)  # Set initial steering angle
    sleep(0.5)  # Delay before starting

    handleLed_waitForButton()  # Handle LED blinking and wait for button press
    defineColorThread.start()  # Start the color definition thread
    motorValue = 0.23  # Set initial motor value

    chronograph = time()  # Start a timer
    print("Loop started!")
    motor.forward(motorValue)  # Start moving forward

    startParkingTime = -1  # Initialize parking time tracker
    
    while True:
        try: 
            motor.forward(motorValue)  # Continue moving forward
            
            lock.acquire()  # Acquire lock for shared values
            shared_values_copy = deepcopy(shared_values)  # Copy shared values for processing
            shared_values[30] = 0  # Reset a specific shared value
            lock.release()  # Release lock

            if shared_values_copy[30] == WALL_WAS_SEEN:
                state.add(WALL)  # Add wall state
                farFromLateralWall = makeDecision(shared_values_copy[0], shared_values_copy[1], shared_values_copy[2], shared_values_copy[3],
                             shared_values_copy[4], shared_values_copy[5], shared_values_copy[6],
                             (shared_values_copy[7], shared_values_copy[8], shared_values_copy[9], shared_values_copy[10]),
                             (shared_values_copy[11], shared_values_copy[12], shared_values_copy[13], shared_values_copy[14]),
                             (shared_values_copy[15], shared_values_copy[16], shared_values_copy[17], shared_values_copy[18]),
                             (shared_values_copy[19], shared_values_copy[20], shared_values_copy[21], shared_values_copy[22]),
                             shared_values_copy[23], shared_values_copy[24], shared_values_copy[25],
                             (shared_values_copy[26], shared_values_copy[27], shared_values_copy[28], shared_values_copy[29])
                             )
                if farFromLateralWall:
                    lastTimeFarFromLateralWall = time()  # Update last time far from lateral wall

                # Check if the IMU has not detected angular velocities for a while
                if imu.noAngularVelocitiesCount > 10:
                    if time() - chronograph > 4.25:
                        state.add(STUCK)  # Add stuck state
                        motor.stop()  # Stop the motor
                        print("Prototype stuck!")
                        sleep(0.5)
                        motor.backward(motorValue)  # Reverse the motor
                        sleep(0.25)
                        for i in range(6):
                            setSteeringAngle(2)  # Adjust steering angle
                            sleep(0.25)
                        motor.stop()  # Stop the motor
                        sleep(0.5)
                    if STUCK in state and startParkingTime != -1 and flagStuckParking:
                        startParkingTime += 5  # Update parking time if stuck

            # Handle line detection and movement (commented out for now)
            if False and LINE in state and direction == CLOCKWISE:
                if time() - lastTimeFarFromLateralWall > 0.6:
                    print("LINE MOVEMENT")
                    setSteeringAngle(predefinedSteeringValueBend * direction)
                    sleep(0.55)
                    setSteeringAngle(steeringAngleForAfterSteering)
                state.discard(LINE)

            # Handle parking logic
            if lap == 3:        
                if startParkingTime == -1:
                    print("I should start parking!")
                    startParkingTime = time()  # Start parking timer
                    if direction == CLOCKWISE:
                        parkingTime += 0.5  # Adjust parking time based on direction
                elif time() - startParkingTime > parkingTime:
                    print("Finished parking!")
                    motor.stop()  # Stop the motor
                    setSteeringAngle(2)  # Adjust steering angle
                    imageProcessing.stop()  # Stop image processing
                    colorSensor.stop = True  # Stop color sensor
                    motor.stop()  # Stop the motor
                    break  # Exit the loop
            
            sleep(sleepTimeForMainCycle)  # Delay for main cycle
        except Exception as exc:
            print(f"Main cycle error: {exc}")  # Print any errors encountered

if __name__ == '__main__':
    loop()  # Start the main loop
    for i in range(25):
        motor.stop()  # Stop the motor
        sleep(0.1)  # Pause briefly
    exit()  # Exit the program
