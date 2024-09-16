from openVariables import *
from differentStates import *
from time import time

# Initialize global variables for steering angles and state
minAngle, maxAngle = -22, 20
state = None
timeLastLine = 0
direction = 0

def setState(_state, _timeLastLine):
    """
    Update the global state and last line detection time.
    
    :param _state: The new state to set.
    :param _timeLastLine: The time when the line was last detected.
    """
    global state, timeLastLine
    state = _state
    timeLastLine = _timeLastLine

def setDirection(_direction):
    """
    Update the global direction.
    
    :param _direction: The new direction to set.
    """
    global direction
    direction = _direction
    
def setMinSteeringAngle(direction):
    """
    Set the minimum and maximum steering angles based on the direction.
    
    :param direction: The direction to adjust steering angles for.
    """
    global minAngle, maxAngle
    if direction == CLOCKWISE:
        maxAngle = 0
    elif direction == ANTICLOCKWISE:
        minAngle = 0
    else:
        minAngle = -20
        maxAngle = 20

def setSteeringAngle(angle, importance=0):
    """
    Set the steering angle while applying constraints and adjustments.
    
    :param angle: The desired steering angle.
    :param importance: Optional parameter to adjust the steering based on importance (default is 0).
    """
    # Adjust angle based on predefined thresholds
    if angle > 14:
        angle += 2
    elif angle > 0:
        angle += 1
    if angle < -14:
        angle -= 4  # Adjust for sharper turns
    elif angle < 0:
        angle -= 3  # Adjust for less sharp turns
    
    # Constrain angle within the min and max angle limits
    if minAngle <= angle <= maxAngle:
        servo.angle = angle + SERVOOFFSET
    elif angle < minAngle:
        servo.angle = minAngle + SERVOOFFSET
    else:
        servo.angle = maxAngle + SERVOOFFSET

    # Special condition for anticlockwise direction
    if direction == ANTICLOCKWISE and (angle < -12 or angle > 12) and time() - timeLastLine < 3.5:
        motor.forward(0.17)