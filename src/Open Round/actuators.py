from openVariables import *  # Import necessary variables/constants from the openVariables module
from time import time  # Import the time function for any timing operations
import serial

# Serial Communication Setup
SERIAL_PORT = "/dev/ttyAMA1"  # Replace with your serial port
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

def sendSerialAngle(angle):
    """Send the steering angle to the servo via serial."""
    with ser:
        ser.write(f"{(angle) + 50 + SERVOOFFSET}".encode('utf-8'))  # Send the angle as a newline-terminated string
        #print(f"Sent angle to serial: {angle}")
        
        
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
            sendSerialAngle(angle)  # Adjust the servo with a predefined offset
        elif angle < -20:  # Cap angle at -20 (too much left)
            sendSerialAngle(-20)
        else:  # Cap angle at +20 (too much right)
            sendSerialAngle(20)
    else:  # If importance is negative, apply stricter limits
        if -15 <= angle <= 15:  # Stricter angle limits for lower importance
            sendSerialAngle(angle)  # Set angle with offset within reduced range
        elif angle < -15:  # Limit left angle
            sendSerialAngle(-15)
        else:  # Limit right angle
            sendSerialAngle(15)

    # If angle is beyond ±14 degrees, initiate forward movement
    if angle < -14 or angle > 14:
        motor.forward(0.18)  # Move forward slowly when turning sharply

# Italian comments explaining the logic for steering:
# 'Per andare a destra l'angolo deve essere negativo'
# (To turn right, the angle must be negative)
# 'Per andare a sinistra l'angolo deve essere positivo'
# (To turn left, the angle must be positive)

def reverse(steeringAngle, sleepTime1=0.8, sleepTime2=0.4):
    '''
    Reverses the motor with a given steering angle.
    
    Parameters:
    - steeringAngle: The angle at which the vehicle reverses.
    - sleepTime1: Duration of the first phase of reversing (not yet implemented).
    - sleepTime2: Duration of the second phase of reversing (not yet implemented).
    
    Note: This function is currently disabled (returns early).
    '''
    
    return  # Function disabled, no operations are executed
    
    # The code below would reverse the steering angle if enabled
    steeringAngle = steeringAngle - 4 if steeringAngle < 0 else steeringAngle + 4
    # First movement: reverse
    setSteeringAngle(steeringAngle)
    motor.backward(0.18)
    sleep(sleepTime1)
    motor.stop()
    # Second movement: spacing
    setSteeringAngle(2)
    motor.backward(motorValue)
    sleep(sleepTime2 / 1.2)
    motor.stop()
    # Third movement: reverse
    setSteeringAngle(-steeringAngle / 2)
    motor.forward(0.18)
    sleep(sleepTime2 / 1.2)
    motor.stop()
    #Fourth movement: moving front
    setSteeringAngle(steeringAngle)
    motor.forward(0.18)
    sleep(sleepTime2 / 1.5)
