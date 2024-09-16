from threading import Thread
from multiprocessing import Process, Lock
from copy import deepcopy
from time import time, sleep
from openVariables import *
from wallRecognitionConstants import *
from finalImageProcessing import ImageProcessing
from brain import makeDecision, setDirection as brainSetDirection, setState as brainSetState
from actuators import setSteeringAngle, reverse

# Pause time for the color sensor to stabilize between readings
pauseTimeColorSensor = 7

from differentStates import *

# Initialize global variables
state = set()  # Set to keep track of current states
timeLastLine = [0]  # List to store the last time a line was detected
brainSetState(state, timeLastLine)  # Initialize the brain state with the current state
parkingTime = 5.0  # Time allocated for parking


def setDirection():
    global direction
    brainSetDirection(direction)  # Update the direction in the brain
    imageProcessing.setDirection(direction)  # Pass the direction to image processing

def sideCounter():
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


def loop():
    global lap, side, updatedSide, state, parkingTime
    global direction, STOP, imageProcessing, imageProcessing_process
    global colorThread
    
    # Initialize threads for color sensor
    colorThread = Thread(target=colorSensor.readDataContinuously)
    
    # Initialize image processing with shared values and lock for thread-safe operations
    lock = Lock()
    shared_values, shared_values_copy = [i for i in range(31)], []
    imageProcessing = ImageProcessing(shared_values, lock, True, True)
    imageProcessing.run()  # Start the image processing

    colorThread.start()  # Start the color sensor thread
    sleep(5)  # Delay for sensor initialization

    print("Setup completed!")
    setSteeringAngle(0)  # Set initial steering angle
    sleep(0.5)  # Delay before starting
    motorValue = 0.23  # Set initial motor value

    chronograph = time()  # Start a timer
    print("Loop started!")
    motor.forward(motorValue)  # Start moving forward
    
    while True:
        pass

if __name__ == '__main__':
    loop()  # Start the main loop
