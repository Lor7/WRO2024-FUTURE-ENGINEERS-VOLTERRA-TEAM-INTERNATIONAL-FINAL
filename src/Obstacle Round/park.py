from math import atan2
from actuators import setSteeringAngle, reverse, simpleReverse
from time import time, sleep
from obstacleRecognitionConstants import CLOCKWISE, ANTICLOCKWISE, FRAME_HEIGHT, FRAME_WIDTH, focalLength, obstacleHeight
import cv2
from openVariables import motor, servo

# Initialize steering angle to 0 degrees
setSteeringAngle(0)

# Constants for frame dimensions
HALF_FRAME_WIDTH = FRAME_WIDTH // 2
HALF_FRAME_HEIGHT = FRAME_HEIGHT // 2

# Global direction and motor value settings
direction = ANTICLOCKWISE
motorValue = 0.23

# Flags to track parking state
gateAlreadyOpened = False
flagAlreadyHere = False

def setDirection(_direction):
    """Set the direction of the robot."""
    global direction
    direction = _direction

def handleParking(magentas, frame=None):
    """
    Handle the parking logic based on the detected magenta blocks.

    Parameters:
        magentas (list): List of detected magenta blocks in the frame.
        frame (optional): The image frame for visualization purposes.
    
    Returns:
        bool: True if the robot should stop, False otherwise.
    """
    global gateAlreadyOpened, flagAlreadyHere
    deltaX, deltaY = 0, 0
    virtualPoint = HALF_FRAME_WIDTH

    # Determine if reversing based on direction
    reverse = False if direction == CLOCKWISE else True

    # Sorting and handling detected magenta blocks
    if len(magentas) == 2:
        magentas.sort(key=lambda x: x[0], reverse=reverse)
        m1 = magentas[0]
        m2 = magentas[1]
    elif len(magentas) == 1:
        # If only one magenta block is detected, adjust its position
        if False:#magentas[0][0] > 240:
            #print("Only one magenta was seen, but two are needed to park")
            m1 = [magentas[0][0] - (3 * magentas[0][2]), magentas[0][1], magentas[0][2], magentas[0][3]]
        else:
            m1 = magentas[0]
        m2 = magentas[0]
    elif len(magentas) > 2:
        magentas.sort(key=lambda x: x[0], reverse=reverse)
        m1 = magentas[0]
        m2 = magentas[-1]
    else:
        print("No magenta blocks were seen")
        return None

    # Estimate distances based on detected block sizes
    estimatedDistanceM1 = obstacleHeight * focalLength / m1[3]
    estimatedDistanceM2 = obstacleHeight * focalLength / m2[3]

    # Handle parking based on the direction
    if direction == CLOCKWISE:
        if m2[0] - m1[0] > 80:
            gateAlreadyOpened = True
            print("Gate opened")
        if m2[0] - m1[0] > 150:
            virtualPoint = (m1[0] + (m2[0] + m1[2]) / 2)
            deltaX = HALF_FRAME_WIDTH - virtualPoint
            print(f"Parking virtual point: {virtualPoint}")
        elif gateAlreadyOpened and m1 == m2 and m1[2] > 100 and m2[3] > 150 and not flagAlreadyHere:
            pass
        else:
            if estimatedDistanceM1 < 45 or not (m1[0] + m1[2] > 240 and m1[1] + m1[3] > 240):
                print("Parking outer point")
                virtualPoint = m2[0] + m2[2] * 1.3
            else:
                print("Parking outer point + width * 2")
                virtualPoint = m2[0] + m2[2] * 2
                
            deltaX = HALF_FRAME_WIDTH - virtualPoint
        deltaY = FRAME_HEIGHT - m2[1] - m2[3]
        
    elif direction == ANTICLOCKWISE:
        if m1[0] - m2[0] > 80:
            print("Gate opened")
            gateAlreadyOpened = True
        if m1[0] - m2[0] > 150:
            virtualPoint = (m2[0] + ((m1[0] - m2[0]) + m2[2]) / 2)
            deltaX = HALF_FRAME_WIDTH - virtualPoint
            print(f"Parking virtual point: {virtualPoint}")
        elif gateAlreadyOpened and m1 == m2 and m1[2] > 100 and m2[3] > 150 and not flagAlreadyHere:
            pass
        else:
            if estimatedDistanceM1 < 45 or not (m1[0] < 400 and m1[1] + m1[3] > 240):
                print("Parking outer point")
                virtualPoint = m2[0] - m2[2] * 0.3
            else:
                print("Parking outer point + width * 1")
                virtualPoint = m2[0] - m2[2] * 1
            
            deltaX = HALF_FRAME_WIDTH - virtualPoint
        deltaY = FRAME_HEIGHT - m2[1] - m2[3]

    # Draw the virtual point on the frame if available
    if frame is not None:
        virtualPoint = int(virtualPoint)
        deltaY = int(deltaY)
        cv2.rectangle(frame, (virtualPoint - 4, deltaY - 4), (virtualPoint + 4, deltaY + 4), (0, 255, 255), 2)

    # Calculate the steering angle based on deltaX and deltaY
    angle = atan2(deltaX, deltaY) * 57.3  # Convert radians to degrees
    steeringAngle = angle / 2.1
    #print(f"Parking steeringAngle: {steeringAngle}")
    setSteeringAngle(steeringAngle)

    # Stop the robot if conditions are met
    if m1[1] + m1[3] > 400 or m2[1] + m2[3] > 400 or ((m1[2] > 240 or m2[2] > 240) and (m2[2] / m2[3] > 1.6 or m1[2] / m1[3] > 1.6)):
        print("STOP")
        return True
    else:
        return False
