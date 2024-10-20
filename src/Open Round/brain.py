from colorama import Fore, Back, Style
from wallRecognitionConstants import (
    CLOCKWISE, ANTICLOCKWISE, FRAME_WIDTH,
    correctionAreaPercentageIn, correctionAreaPercentageMid, correctionAreaPercentageOut,
    correctionAreaPercentageFront, correctionAreaPercentageLower, correctionAreaPercentageFront2
)
from actuators import setSteeringAngle, reverse
from time import time

# Global variables for storing direction, steering corrections, and state tracking
direction = 0
softCorrection = 8.5  # Soft adjustment for steering
state = None
timeLastLine = None

# Overwriting print to disable debug output
print = lambda *x, **y: None

# Function to update the state and time of the last line detection
def setState(_state, _timeLastLine):
    global state, timeLastLine
    state = _state
    timeLastLine = _timeLastLine

# Function to set the current driving direction
def setDirection(_direction):
    global direction
    direction = _direction

# Decision-making function for robot navigation
# Input includes various detected color areas, line coordinates, and flags
def makeDecision(coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn,
                 coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn, coloredAreaFront,
                 leftRect, rightRect, leftLine, rightLine,
                 coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2,
                 centralLine):
    
    # Initialize variables to track wall proximity, line slopes, and field status
    centralSlope, leftSlope, rightSlope = 0, 0, 0
    targetPoint = None
    nearLeftCorner, nearRightCorner = False, False
    fieldModified, fieldStatus = False, None
    closeToWall = True

    # Boolean flags for detecting presence of walls based on color areas
    leftFlagOut = coloredAreaLeftOut > correctionAreaPercentageOut
    leftFlagMid = coloredAreaLeftMid > correctionAreaPercentageMid
    leftFlagIn = coloredAreaLeftIn > correctionAreaPercentageIn
    rightFlagOut = coloredAreaRightOut > correctionAreaPercentageOut
    rightFlagMid = coloredAreaRightMid > correctionAreaPercentageMid
    rightFlagIn = coloredAreaRightIn > correctionAreaPercentageIn
    frontFlag = coloredAreaFront > correctionAreaPercentageFront
    lowerRightFlag = coloredAreaLowerRight > correctionAreaPercentageLower
    lowerLeftFlag = coloredAreaLowerLeft > correctionAreaPercentageLower
    frontFlag2 = coloredAreaFront2 > correctionAreaPercentageFront2

    # Calculate slopes (angle of inclination) for left, right, and central lines
    # If the slope is steep enough, we consider the robot to be near a corner
    if leftLine[0] - leftLine[2] != 0:
        leftSlope = abs((leftLine[1] - leftLine[3]) / (leftLine[0] - leftLine[2]))
        if leftSlope > 3.2:
            nearLeftCorner = True
    if rightLine[0] - rightLine[2] != 0:
        rightSlope = abs((rightLine[1] - rightLine[3]) / (rightLine[0] - rightLine[2]))
        if rightSlope > 3.2:
            nearRightCorner = True
    if centralLine[0] - centralLine[2] != 0:
        centralSlope = abs((centralLine[1] - centralLine[3]) / (centralLine[0] - centralLine[2]))

    # Check if the robot is close to a wall, based on flags and the current direction
    if direction == CLOCKWISE and not(rightFlagMid or lowerRightFlag or rightFlagOut):
        closeToWall = False
    elif direction == ANTICLOCKWISE and not(leftFlagMid or lowerLeftFlag or leftFlagOut):
        closeToWall = False
    elif direction == 0 and not(rightFlagMid or lowerRightFlag or rightFlagOut) and not(leftFlagMid or lowerLeftFlag or leftFlagOut):
        closeToWall = False

    # Conditional logic for adjusting steering based on detected walls and flags
    # Handles cases like detecting obstacles in front or corners
    if direction == 0 and frontFlag2 and leftFlagIn and leftFlagOut and leftFlagMid and not lowerRightFlag and lowerLeftFlag and 0.25 < leftSlope < 1.3:
        print("Grey zone, steering right")
        setSteeringAngle(-17, importance=1)

    # If robot is going in the wrong direction, adjust steering forcefully
    elif (direction == 0 or direction == CLOCKWISE) and frontFlag2 and lowerLeftFlag and leftFlagMid and leftFlagOut and leftSlope > 0.6 and rightSlope < 0.7 and rightFlagMid and rightFlagIn:
        print("Totally wrong direction! Swerving right")
        if frontFlag:
            setSteeringAngle(-18, importance=1)
        else:
            setSteeringAngle(-17, importance=1)

    # Continue logic for detecting walls, refining direction, avoiding corners, and making steering corrections
    # This block contains numerous if-else conditions that adjust the robot's steering angle based on detected inputs
    # These corrections aim to avoid walls, maintain a straight path, or navigate around tight corners

    elif (direction == ANTICLOCKWISE) and frontFlag2 and lowerRightFlag and rightFlagMid and rightFlagOut and rightSlope > 0.6 and leftSlope < 0.7 and leftFlagMid and leftFlagIn:
        print("Totally wrong direction! Swerwing left")
        if frontFlag:
            setSteeringAngle(18, importance = 1)
        else:
            setSteeringAngle(17, importance = 1)
    elif (direction == ANTICLOCKWISE or direction == 0) and leftSlope > 2 and lowerLeftFlag and not(leftFlagMid) and (rightFlagMid or rightFlagOut) and not(lowerRightFlag):
        print("Steering angle deflected right")
        setSteeringAngle(-13, importance = 1)
    elif (direction == CLOCKWISE or direction == 0) and rightSlope > 2 and lowerRightFlag and not(rightFlagMid) and (leftFlagMid or leftFlagOut) and not(lowerLeftFlag):
        print("Steering angle deflected left")
        setSteeringAngle(13, importance = 1)
    elif direction == ANTICLOCKWISE and lowerLeftFlag and leftFlagOut and rightFlagMid and rightFlagOut and rightSlope < 0.55 and (lowerRightFlag and (leftRect[0]+leftRect[1] > 290 or leftSlope == 0)):
        setSteeringAngle(-15, importance = 1)
        print("Steering angle brought to the right")
    elif direction == CLOCKWISE and lowerRightFlag and rightFlagOut and leftFlagMid and leftFlagOut and leftSlope < 0.55 and not(centralSlope < 0.28 and (frontFlag or frontFlag2) and leftFlagIn and lowerLeftFlag and rightFlagMid and rightFlagIn):
        setSteeringAngle(15, importance = 1)
        print("Steering angle brought to the left")
    elif direction == CLOCKWISE and (frontFlag or frontFlag2) and (time() - timeLastLine[0] < 4) and 0 < centralSlope < 0.25 and 0 < rightSlope < 0.25 and (rightFlagMid or leftFlagMid) and (rightFlagIn or leftFlagIn) and rightRect[0] + rightRect[1] < 620:
        setSteeringAngle(-18, importance = 1)
        print("Steering forced clockwise")
    elif direction == ANTICLOCKWISE and (frontFlag) and (time() - timeLastLine[0] < 4) and 0 < centralSlope < 0.25 and 0 < leftSlope < 0.25 and (rightFlagMid or leftFlagMid) and (rightFlagIn or leftFlagIn) and (rightFlagOut or leftFlagOut) and leftRect[0] < 20:
        setSteeringAngle(18, importance = 1)
        print("Steering forced anticlockwise")
    elif (direction == 0 or direction == ANTICLOCKWISE) and (frontFlag2) and (time() - timeLastLine[0] > 5) and (rightSlope > 0.6 and leftSlope > 0.6) and lowerRightFlag and lowerLeftFlag and leftFlagMid and leftFlagOut and rightFlagMid and rightFlagOut and not(leftFlagIn):
        print("Direction refinement: -2")
        setSteeringAngle(-2)
    elif (direction == 0 or direction == ANTICLOCKWISE) and (frontFlag2) and (time() - timeLastLine[0] > 5) and (rightSlope > 0.6 and leftSlope > 0.6) and lowerLeftFlag and not(rightFlagIn) and rightFlagMid and leftFlagMid and leftFlagOut and leftFlagIn:
        print("Direction correction: -10")
        setSteeringAngle(-10)
    elif (direction == ANTICLOCKWISE) and (time() - timeLastLine[0] < 5) and not(leftFlagIn or leftFlagOut or leftFlagMid) and leftSlope > 2 and rightSlope < 0.3 and rightFlagMid and rightFlagIn and rightFlagOut:
        print("Direction improvement: -9")
        setSteeringAngle(-9)
    elif direction == CLOCKWISE and time() - timeLastLine[0] < 5 and rightSlope > 2 and leftSlope < 0.3 and not(rightFlagIn or rightFlagOut or rightFlagMid) and leftFlagMid and leftFlagIn and leftFlagOut:
        print("Direction improvement: 9")
        setSteeringAngle(9)
    elif direction == ANTICLOCKWISE and time() - timeLastLine[0] < 2.5 and leftSlope > 2:
        print("Corner avoidance: -9")
        setSteeringAngle(-9)
    elif (frontFlag or frontFlag2) and leftFlagMid and leftFlagOut and lowerLeftFlag and not(rightFlagMid or rightFlagOut or lowerRightFlag):
        print("Too close to the left wall")
        setSteeringAngle(-14)
    elif (frontFlag or frontFlag2) and rightFlagMid and rightFlagOut and lowerRightFlag and not(leftFlagMid or leftFlagOut or lowerLeftFlag) and not(leftSlope == 0 and time() - timeLastLine[0] < 2.5):
        print("Too close to the right wall")
        setSteeringAngle(13)
    elif frontFlag and ( not(rightFlagIn) and not(rightFlagOut) and rightSlope == 0 and leftFlagIn):
        print(f"{Style.DIM}{Fore.RED}Steering CLOCKWISE because too close to the front wall{Style.RESET_ALL}")
        setSteeringAngle(-18, importance = 1)
    elif frontFlag and ( not(leftFlagIn) and not(leftFlagOut) and leftSlope == 0 and rightFlagIn):
        print(f"{Style.DIM}{Fore.RED}Steering ANTICLOCKWISE because too close to the front wall{Style.RESET_ALL}")
        setSteeringAngle(18, importance = 1) #
    elif nearRightCorner and leftSlope > 0.65:
        setSteeringAngle(softCorrection)
        print(Fore.RED + "Go left because too close to the right corner" + Style.RESET_ALL)
    elif nearLeftCorner and rightSlope > 0.65:
        setSteeringAngle(-softCorrection)
        print(Fore.RED + "Go right because too close to the left corner" + Style.RESET_ALL)
    elif leftFlagMid and rightFlagMid:
        if leftFlagIn and rightFlagIn and frontFlag:
            if 0 < leftSlope < 0.25 and direction == ANTICLOCKWISE:# and rightSlope > 0:
                # Go as ANTICLOCKWISE possible, because facing the wall
                print(Fore.RED + "Go as ANTICLOCKWISE possible, because facing the wall" + Style.RESET_ALL)
                setSteeringAngle(18, importance = 1)
            elif 0 < rightSlope < 0.25 and direction == CLOCKWISE:# and leftSlope > 0:
                # Go as CLOCKWISE possible, because facing the wall
                print(Fore.RED + "Go as CLOCKWISE possible, because facing the wall" + Style.RESET_ALL)
                setSteeringAngle(-18, importance = 1)
            else:
                # Go as CLOCKWISE or ANTICLOCKWISE possible, because facing the wall
                print(Fore.RED + "Go as CLOCKWISE or ANTICLOCKWISE possible, because facing the wall" + Style.RESET_ALL)
                setSteeringAngle(-18 * direction, importance = 1)
        elif leftFlagIn and not(rightFlagIn):
            if not(direction == ANTICLOCKWISE and time() - timeLastLine[0] < 4.5 and 0 < leftSlope < 0.65 and rightSlope > 0.35 and (lowerRightFlag or rightFlagOut) and (frontFlag or frontFlag2 or centralSlope != 0)):
                # Go a bit right
                print(Fore.RED + "Go a bit right" + Style.RESET_ALL)
                setSteeringAngle(-8)
            else:
                print("Go a bit left from right")
                setSteeringAngle(8)
        elif rightFlagIn and not(leftFlagIn):
            if not(direction == ANTICLOCKWISE and (leftSlope > 1 or (not(rightFlagOut or frontFlag) and centralSlope < 0.45))):
                if not(direction == CLOCKWISE and time() - timeLastLine[0] < 3.5 and (frontFlag or frontFlag2) and leftSlope > 0.3 and not(lowerRightFlag)):#
                    if not((direction == 0) and frontFlag2 and leftSlope > 0.45 and not(lowerRightFlag) and leftRect[0]+leftRect[1]<105 and rightRect[1] < 20):
                        # Go a bit left
                        print(Fore.RED + "Go a bit left" + Style.RESET_ALL)
                        setSteeringAngle(9)#8
                    else:
                        print(Fore.CYAN + "Go decisely right from a bit left" + Style.RESET_ALL)
                        setSteeringAngle(-15)
                else:
                    print("Go right from a bit left")
                    setSteeringAngle(-15)
            else:
                #Go a bit righte from going a bit left
                print(Fore.RED + "Go a bit right from going a bit left" + Style.RESET_ALL)
                setSteeringAngle(-12)
        else:
            if leftSlope > 0.8 and rightSlope < 0.55:
                if not(direction == ANTICLOCKWISE and (leftSlope > 1 and frontFlag2 and not(frontFlag))):
                    #Go slightly right
                    print(Style.DIM + Fore.LIGHTYELLOW_EX + "Go slightly right" + Style.RESET_ALL)
                    setSteeringAngle(-7)
                else:
                    #Go right from slightly right
                    print(Style.DIM + Fore.LIGHTYELLOW_EX + "Go right from slightly right" + Style.RESET_ALL)
                    setSteeringAngle(-12)    
            elif rightSlope > 0.8 and leftSlope < 0.55:
                #
                if not(direction == CLOCKWISE and time() - timeLastLine[0] < 4.5 and leftSlope > 0.28 and rightSlope > 0.35 and (frontFlag or frontFlag2) and (rightFlagIn and leftFlagIn and leftFlagMid and rightFlagMid)):
                    # Go slightly left
                    print(Style.DIM + Fore.LIGHTYELLOW_EX + "Go slightly left" + Style.RESET_ALL)
                    setSteeringAngle(8)#7
                else:
                    #Go right from slightly left
                    print("Go right from slightly left" + Style.RESET_ALL)
                    setSteeringAngle(-15)    
            else:
                #
                if not(direction == ANTICLOCKWISE and time() - timeLastLine[0] < 4.5 and (frontFlag or frontFlag2 or centralSlope != 0) and centralSlope < 0.35 and (rightFlagIn or rightFlagOut or leftFlagMid) and leftFlagIn):
                    if not(direction == CLOCKWISE and time() - timeLastLine[0] < 4.5 and (frontFlag or frontFlag2 or centralSlope != 0) and (leftFlagIn or leftFlagOut or rightFlagMid) and (rightFlagIn or (lowerLeftFlag and rightRect[1] < 30))):
                        # Go straight
                        print(Style.DIM + Fore.YELLOW + "Go straight" + Style.RESET_ALL)
                        if direction == 0:
                            setSteeringAngle(2)#0
                        elif direction == CLOCKWISE:
                            setSteeringAngle(1)
                        elif direction == ANTICLOCKWISE:
                            setSteeringAngle(4)
                    else:
                        print("Now go right, before you would've gone straight")
                        setSteeringAngle(-15)
                else:
                    print("Now go left, before you would've gone straight")
                    setSteeringAngle(12)
    elif leftFlagMid and not(rightFlagMid) and leftSlope < 1: # Prima era leftSlope < 0.8
        if leftFlagIn and rightFlagIn and not(direction == ANTICLOCKWISE and leftSlope == 0 and not(lowerLeftFlag)):
            # Go as right possible, because facing the wall 
            print(Fore.RED + "Go as right possible, because facing the wall" + Style.RESET_ALL)
            setSteeringAngle(-18, importance = 1)
            #.5)
        elif (leftFlagIn) and not(rightFlagIn) and not(lowerRightFlag) and not(direction == ANTICLOCKWISE and leftSlope < 0.3 and centralSlope < 0.3 and (frontFlag or frontFlag2)):## Questione della distanza dal centro
            # Go right
            print(Fore.RED + "Go right" + Style.RESET_ALL)
            setSteeringAngle(-12.5)
        elif not(rightFlagIn) and rightSlope > leftSlope * 1.2:#rightFlagOut or <-- dentro al not()
            if not(direction == ANTICLOCKWISE and not(frontFlag) and frontFlag2 and not(lowerLeftFlag)):
                setSteeringAngle(-7)
                print("Shift a little right")
            else:
                setSteeringAngle(11)
                print("Shift a little left from right")
        else:  
            # Go straight
            print(Style.DIM + Fore.GREEN + "Go straight!" + Style.RESET_ALL)
            if direction == 0:
                setSteeringAngle(2)#0
            elif direction == CLOCKWISE:
                setSteeringAngle(1)
            elif direction == ANTICLOCKWISE:
                setSteeringAngle(4)
    elif rightFlagMid and not(leftFlagMid) and rightSlope < 1: # Prima era rightSlope < 0.8
        if leftFlagIn and rightFlagIn:
            if not(direction == CLOCKWISE and time() - timeLastLine[0] < 3.5 and not(lowerRightFlag or frontFlag) and rightSlope < 0.45): #
                # Go as left possible, because facing the wall 
                print(Fore.RED + "Go as left possible, because facing the wall" + Style.RESET_ALL)
                setSteeringAngle(18, importance = 1)
            else:
                print(Fore.CYAN + "Adapt going right" + Style.RESET_ALL)
                setSteeringAngle(-12)
        elif (rightFlagIn or lowerRightFlag) and not(leftFlagIn) and not(lowerLeftFlag): ## Questione della distanza dal centro
            if not((direction == 0 or direction == CLOCKWISE) and frontFlag2 and leftSlope > 0.45 and not(lowerRightFlag) and leftRect[0]+leftRect[1]<105 and rightRect[1] < 20):
                # Go left  
                print(Fore.RED + "Go left" + Style.RESET_ALL)
                setSteeringAngle(13.5)#12.5
            else:
                print(Fore.CYAN + "Go right instead of left" + Style.RESET_ALL)
                setSteeringAngle(-15)
        elif not(leftFlagIn) and leftSlope > rightSlope * 1.2: #leftFlagOut or 
            if not(direction == ANTICLOCKWISE and not(frontFlag) and frontFlag and leftSlope > 1):
                # 
                if not((direction == 0 or (direction == CLOCKWISE and time() - timeLastLine[0] < 3.5)) and frontFlag2 and not(lowerRightFlag or leftFlagIn or leftFlagMid or frontFlag) ):
                    setSteeringAngle(8)#7
                    print("Shift a little left")
                else:
                    print(Fore.CYAN + "Fine tune right" + Style.RESET_ALL)
                    setSteeringAngle(-11)
            else:
                setSteeringAngle(-12)
                print("Change direction to right, from shifting left")
        else:
            if not(False): 
                # Go straight
                print(Style.DIM + Fore.GREEN + "Go straight" + Style.RESET_ALL)
                if direction == 0:
                    setSteeringAngle(2)#0
                elif direction == CLOCKWISE:
                    setSteeringAngle(1)
                elif direction == ANTICLOCKWISE:
                    setSteeringAngle(4)
            else:
                print(Fore.CYAN + "Rearrange right" + Style.RESET_ALL)
    elif lowerRightFlag and not(rightFlagIn) and not(leftFlagIn) and frontFlag2:
        setSteeringAngle(10)#9
        print(f"{Fore.RED}Moving left beacuse too close to the corner{Style.RESET_ALL}")
    elif lowerLeftFlag and not(rightFlagIn) and not(leftFlagIn)  and frontFlag2:
        setSteeringAngle(-9)
        print(f"{Fore.RED}Moving right beacuse too close to the corner{Style.RESET_ALL}")
    else:
        if rightSlope != 0 and (leftSlope < rightSlope * 1.1) and (leftFlagOut and not(rightFlagOut)):
            # Go a bit right
            print(Fore.RED + "Translate a bit right" + Style.RESET_ALL)
            setSteeringAngle(-7.5)      
        elif leftSlope != 0 and (rightSlope < leftSlope * 1.1) and (rightFlagOut and not(leftFlagOut)):
            if not(direction == 0 and frontFlag2 and not(lowerRightFlag or leftFlagIn or leftFlagMid or frontFlag)):
                # Go a bit left
                print(Fore.RED + "Translate a bit left" + Style.RESET_ALL)
                setSteeringAngle(8.5)#7.5
            else:
                print(Fore.CYAN + "Alter to right" + Style.RESET_ALL)
                setSteeringAngle(-11)
        elif rightSlope != 0 and leftSlope != 0 and rightSlope > leftSlope and rightFlagIn and rightFlagOut and lowerRightFlag and not(leftFlagOut and leftFlagMid):
            print(Fore.MAGENTA + "Advance left" + Style.RESET_ALL)
            setSteeringAngle(8.5) 
        elif rightSlope != 0 and leftSlope != 0 and rightSlope < leftSlope and leftFlagIn and leftFlagOut and lowerLeftFlag and not(rightFlagOut and rightFlagMid):
            print(Fore.MAGENTA + "Advance right" + Style.RESET_ALL)
            setSteeringAngle(-7.5) 
        else: 
            if not((direction == 0 or (direction == CLOCKWISE and time() - timeLastLine[0] < 3.5)) and rightSlope < 0.25 and frontFlag2 and (leftSlope == 0 or leftSlope > 0.35) and not(lowerRightFlag or leftFlagIn or leftFlagMid or frontFlag)):
                print(Style.BRIGHT + Fore.GREEN + "Everything fine! Going straight!" + Style.RESET_ALL)
                if direction == 0:
                    setSteeringAngle(2)#0
                elif direction == CLOCKWISE:
                    setSteeringAngle(1)
                elif direction == ANTICLOCKWISE:
                    setSteeringAngle(4)
            else:
                print(Fore.CYAN + "Adjusting right" + Style.RESET_ALL) 
                setSteeringAngle(-11)
    
    
    # Return value says whether the prototype is far (= True) or close (= False) respect to the wall
    return not(closeToWall)