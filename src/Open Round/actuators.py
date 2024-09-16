from openVariables import *  # Import necessary variables/constants from the openVariables module
from time import time  # Import the time function for any timing operations

def setSteeringAngle(angle, importance=0):
    '''
    Adjusts the steering angle of the servo motor.
    
    Parameters:
    - angle: Desired steering angle (in degrees).
    - importance: Determines the range of angle limits. 
    Higher values allow larger adjustments, while negative values limit them.
    '''
    
    # Fine-tune angle depending on its range to avoid abrupt movements
    if angle > 14:
        angle += 2  # Slight increase for larger positive angles (sharp right turn)
    elif angle > 0:
        angle += 1  # Slight increase for small positive angles (right turn)
    elif angle < -14:
        angle -= 2  # Slight decrease for larger negative angles (sharp left turn)
    elif angle < 0:
        angle -= 1  # Slight decrease for small negative angles (left turn)
    
    # Apply angle adjustments based on the importance level
    if importance > -1:  # If normal or higher importance
        if -20 <= angle <= 20:  # Ensure the angle is within the safe range
            servo.angle = angle + SERVOOFFSET  # Adjust the servo with a predefined offset
        elif angle < -20:  # Cap angle at -20 (too much left)
            servo.angle = -20 + SERVOOFFSET
        else:  # Cap angle at +20 (too much right)
            servo.angle = 20 + SERVOOFFSET
    else:  # If importance is negative, apply stricter limits
        if -15 <= angle <= 15:  # Stricter angle limits for lower importance
            servo.angle = angle + SERVOOFFSET  # Set angle with offset within reduced range
        elif angle < -15:  # Limit left angle
            servo.angle = -15 + SERVOOFFSET
        else:  # Limit right angle
            servo.angle = 15 + SERVOOFFSET

    # If angle is beyond ±14 degrees, initiate forward movement
    if angle < -14 or angle > 14:
        motor.forward(0.18)  # Move forward slowly when turning sharply

# Italian comments explaining the logic for steering:
# 'Per andare a destra l'angolo deve essere negativo'
# (To turn right, the angle must be negative)
# 'Per andare a sinistra l'angolo deve essere positivo'
# (To turn left, the angle must be positive)

