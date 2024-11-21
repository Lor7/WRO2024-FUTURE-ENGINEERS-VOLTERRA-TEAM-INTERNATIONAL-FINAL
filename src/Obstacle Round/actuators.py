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

def simpleReverse(steeringAngle):
    """
    Perform a simple reverse maneuver with the specified steering angle.
    
    :param steeringAngle: The steering angle to set during the reverse maneuver.
    """
    state.add(SIMPLE_REVERSE)  # Add simple reverse state
    motor.stop()  # Stop the motor
    sleep(0.3)  # Pause
    motor.backward(motorValue)  # Move backward
    sleep(0.3)  # Pause
    motor.stop()  # Stop the motor
    setSteeringAngle(steeringAngle)  # Set steering angle
    motor.backward(motorValue)  # Move backward again
    sleep(0.8)  # Pause
    motor.stop()  # Stop the motor
    setSteeringAngle(2)  # Reset steering angle
    state.discard(SIMPLE_REVERSE)  # Remove simple reverse state

def reverse(steeringAngle, sleepTime1=0.8, sleepTime2=0.4):
    """
    Perform a complex reverse maneuver with the specified steering angle.
    
    :param steeringAngle: The steering angle to set during the reverse maneuver.
    :param sleepTime1: Time to sleep during the initial reverse (default is 0.8 seconds).
    :param sleepTime2: Time to sleep during the distancing and second reverse (default is 0.4 seconds).
    """
    # Adjust steering angle for reverse
    steeringAngle = steeringAngle - 4 if steeringAngle < 0 else steeringAngle + 4
    
    # INITIAL REVERSE
    setSteeringAngle(steeringAngle)  # Set steering angle for initial reverse
    motor.backward(0.18)  # Move backward
    sleep(sleepTime1)  # Pause
    
    motor.stop()  # Stop the motor
    
    # DISTANCING
    setSteeringAngle(2)  # Set neutral steering angle
    motor.backward(motorValue)  # Move backward to distance
    sleep(sleepTime2 / 1.2)  # Pause
    
    motor.stop()  # Stop the motor
    
    # SECOND REVERSE
    setSteeringAngle(-steeringAngle / 2)  # Adjust steering angle for second reverse
    motor.forward(0.18)  # Move forward
    sleep(sleepTime2 / 1.2)  # Pause
    
    motor.stop()  # Stop the motor
    
    # FRONT
    setSteeringAngle(steeringAngle)  # Set steering angle for forward movement
    motor.forward(0.18)  # Move forward
    sleep(sleepTime2 / 1.5)  # Pause

def U_turn(left, seconds=2.5):
    """
    Perform a U-turn in the specified direction.
    
    :param left: Boolean indicating whether to turn left (True) or right (False).
    :param seconds: Duration of the U-turn (default is 2.5 seconds).
    """
    print(f"U turn, seconds: {seconds}")
    if left:
        setSteeringAngle(20)  # Set steering angle for left U-turn
        print(f"U turn LEFT")
    else:
        setSteeringAngle(-20)  # Set steering angle for right U-turn
        print(f"U turn RIGHT")
    
    sleep(0.5)  # Pause to complete the turn setup
    motor.forward(0.2)  # Move forward during the U-turn
    sleep(seconds)  # Duration of the U-turn
    
    motor.stop()  # Stop the motor
    setSteeringAngle(-2)  # Reset steering angle
    motor.backward(0.2)  # Move backward to complete the maneuver
    sleep(1)  # Pause
    
    motor.stop()  # Stop the motor
