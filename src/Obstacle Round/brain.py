# Import necessary libraries and modules
from colorama import Fore, Back, Style  # For colored terminal text
from wallRecognitionConstants import (
    CLOCKWISE, ANTICLOCKWISE, FRAME_WIDTH, 
    correctionAreaPercentageIn, correctionAreaPercentageMid, 
    correctionAreaPercentageOut, correctionAreaPercentageFront, 
    correctionAreaPercentageLower, correctionAreaPercentageFront2
)
from actuators import setSteeringAngle, reverse, setMinSteeringAngle
from obstacleRecognitionConstants import GREENBLOCKID, REDBLOCKID
from time import time

# Initialize global variables
direction = 0
softCorrection = 8.5
state = None
timeLastLine = [0]
timeLastLineConstant = 3.5

# Disable print statements for debugging
print = lambda *x, **y: None

# Function to set the state and last line time
def setState(_state, _timeLastLine):
    global state, timeLastLine
    state = _state
    timeLastLine = _timeLastLine

# Function to set the direction
def setDirection(_direction):
    global direction
    direction = _direction


# Function to make decisions based on various inputs
def makeDecision(coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn, 
                coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn, coloredAreaFront,
                leftRect, rightRect, leftLine, rightLine,
                coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2,
                 centralLine, lastObstacle
    ):
    # Initialize local variables
    centralSlope, leftSlope, rightSlope, targetPoint, nearLeftCorner, nearRightCorner = 0, 0, 0, None, False, False
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
    closeToWall = True

    # Uncomment these lines to debug the flag status
    #print(f"{Style.DIM}{Fore.GREEN}", leftFlagOut, leftFlagMid, leftFlagIn, rightFlagIn, rightFlagMid, rightFlagOut, f"{Style.RESET_ALL}", sep = " -> ")
    #print(f"{Style.DIM}{Fore.GREEN}Front flag: {frontFlag}, Front flag2: {frontFlag2}, lower left: {lowerLeftFlag}, lower right: {lowerRightFlag}{Style.RESET_ALL}")

    # Calculate slope coefficients to determine if near corners
    if leftLine[0] - leftLine[2] != 0:
        leftSlope = abs((leftLine[1] - leftLine[3]) / (leftLine[0] - leftLine[2]))
        # Uncomment to debug the left slope
        #print(f"{Style.DIM}{Fore.YELLOW}Left slope: {leftSlope}{Style.RESET_ALL}")
        if leftSlope > 3.2:
            nearLeftCorner = True
    if rightLine[0] - rightLine[2] != 0:
        rightSlope = abs((rightLine[1] - rightLine[3]) / (rightLine[0] - rightLine[2]))
        # Uncomment to debug the right slope
        #print(f"{Style.DIM}{Fore.YELLOW}Right slope: {rightSlope}{Style.RESET_ALL}")
        if rightSlope > 3.2:
            nearRightCorner = True
    # Uncomment to debug corner proximity
    #print(f"{Style.BRIGHT}{Fore.YELLOW}Near left corner: {nearLeftCorner}, near right corner: {nearRightCorner}{Style.RESET_ALL}")
    if centralLine[0] - centralLine[2] != 0:
        centralSlope = abs((centralLine[1] - centralLine[3]) / (centralLine[0] - centralLine[2]))
        # Uncomment to debug the central slope
        ##print(f"{Style.DIM}{Fore.YELLOW}Right slope: {rightSlope}{Style.RESET_ALL}")

    # Determine if close to the wall based on direction and flags
    if (direction == CLOCKWISE and not (rightFlagMid or (lowerRightFlag or rightFlagOut))):
        closeToWall = False
    elif (direction == ANTICLOCKWISE and not (leftFlagMid or (lowerLeftFlag or leftFlagOut))):
        closeToWall = False
    
    # Make decisions based on obstacles, flags, and time
    if (direction == CLOCKWISE or direction == 0) and lastObstacle[1] == GREENBLOCKID and frontFlag and (rightFlagIn or leftFlagIn) and (
        (time() - lastObstacle[0] < 2.5) or (time() - lastObstacle[0] < 4.2 and time() - timeLastLine[0] < 6.5)):
        print("Green-clockwise case")
        setSteeringAngle(-18, importance=1)
    elif (direction == ANTICLOCKWISE or direction == 0) and lastObstacle[1] == REDBLOCKID and frontFlag and (rightFlagIn or leftFlagIn) and (
        (time() - lastObstacle[0] < 2.5) or (time() - lastObstacle[0] < 4.2 and time() - timeLastLine[0] < 6.5)):
        print("Red-anticlockwise case")
        setSteeringAngle(18, importance=1)
        
    elif (direction == 0 or direction == CLOCKWISE) and frontFlag2 and lowerLeftFlag and leftFlagMid and leftFlagOut and leftSlope > 0.6 and rightSlope < 0.7 and rightFlagMid and rightFlagIn:
        print("Totally wrong direction! Swerwing right")
        if frontFlag:
            setSteeringAngle(-18, importance = 1)
        else:
            setSteeringAngle(-17, importance = 1)
    elif (direction == ANTICLOCKWISE or direction == 0) and leftSlope > 2 and lowerLeftFlag and not(leftFlagMid) and (rightFlagMid or rightFlagOut) and not(lowerRightFlag):
        print("Steering angle deflected right")
        setSteeringAngle(-13, importance = 1)
    elif direction == CLOCKWISE and rightSlope > 2 and lowerRightFlag and not(rightFlagMid) and (leftFlagMid or leftFlagOut) and not(lowerLeftFlag):
        print("Steering angle deflected left")
        setSteeringAngle(13, importance = 1)
    elif direction == ANTICLOCKWISE and lowerLeftFlag and leftFlagOut and rightFlagMid and rightFlagOut and rightSlope < 0.55 and (lowerRightFlag and (leftRect[0]+leftRect[1] > 290 or leftSlope == 0)) and not(leftFlagMid and leftFlagIn and rightFlagIn and lowerRightFlag and leftSlope < 0.25):
        setSteeringAngle(-15, importance = 1)
        print("Steering angle brought to the right")
    elif direction == CLOCKWISE and lowerRightFlag and rightFlagOut and leftFlagMid and leftFlagOut and leftSlope < 0.55 and not(centralSlope < 0.28 and (frontFlag or frontFlag2) and leftFlagIn and lowerLeftFlag and rightFlagMid and rightFlagIn) and not(centralSlope < 0.25 and (leftSlope < 0.25 or not(lowerLeftFlag)) and rightSlope < 0.25):
        setSteeringAngle(15, importance = 1)
        print("Steering angle brought to the left")
    elif direction == CLOCKWISE and (frontFlag or frontFlag2) and (time() - timeLastLine[0] < 4) and 0 < centralSlope < 0.25 and 0 < rightSlope < 0.25 and (rightFlagMid or leftFlagMid) and (rightFlagIn or leftFlagIn) and rightRect[0] + rightRect[1] < 620:
        setSteeringAngle(-18, importance = 1)
        print("Steering forced clockwise")
    elif direction == ANTICLOCKWISE and (frontFlag) and (time() - timeLastLine[0] < 4) and 0 < centralSlope < 0.25 and 0 < leftSlope < 0.25 and (rightFlagMid or leftFlagMid) and (rightFlagIn or leftFlagIn) and (rightFlagOut or leftFlagOut) and leftRect[0] < 20:
        setSteeringAngle(18, importance = 1)
        print("Steering forced anticlockwise")
    elif direction == ANTICLOCKWISE and (frontFlag or frontFlag2) and (time() - lastObstacle[0] < 2 and lastObstacle[1] == GREENBLOCKID) and leftSlope != 0 and rightSlope == 0 and leftFlagMid and leftFlagIn and leftFlagOut and lowerLeftFlag and not(rightFlagMid or rightFlagOut or rightFlagIn or lowerRightFlag):
        print("Going against the center, steer right")
        setSteeringAngle(-13)
    elif direction == CLOCKWISE and (frontFlag or frontFlag2) and (time() - lastObstacle[0] < 2 and lastObstacle[1] == REDBLOCKID) and leftSlope == 0 and rightSlope != 0 and not(leftFlagMid or leftFlagIn or leftFlagOut or lowerLeftFlag) and rightFlagMid and rightFlagOut and rightFlagIn and lowerRightFlag:
        print("Going against the center, steer left")
        setSteeringAngle(13)
    elif (frontFlag or frontFlag2) and leftFlagMid and leftFlagOut and lowerLeftFlag and not(rightFlagMid or rightFlagOut or lowerRightFlag):
        print("Too close to the left wall")
        setSteeringAngle(-13, importance = 1)
    elif (frontFlag or frontFlag2) and rightFlagMid and rightFlagOut and lowerRightFlag and not(leftFlagMid or leftFlagOut or lowerLeftFlag):
        print("Too close to the right wall")
        setSteeringAngle(13, importance = 1)
    elif frontFlag and ( not(rightFlagIn) and not(rightFlagOut) and rightSlope == 0 and leftFlagIn):
        if time() - timeLastLine[0] < timeLastLineConstant and direction == ANTICLOCKWISE:
            print(f"{Style.DIM}{Fore.RED}Steering ANTICLOCKWISE, advice from the line {Style.RESET_ALL}")
            setSteeringAngle(18)
        else:
            print(f"{Style.DIM}{Fore.RED}Steering CLOCKWISE because too close to the front wall{Style.RESET_ALL}")
            setSteeringAngle(-18, importance = 0)
    elif frontFlag and ( not(leftFlagIn) and not(leftFlagOut) and leftSlope == 0 and rightFlagIn):
        if time() - timeLastLine[0] < timeLastLineConstant and direction == CLOCKWISE:
            print(f"{Style.DIM}{Fore.RED}Steering CLOCKWISE, advice from the line {Style.RESET_ALL}")
            setSteeringAngle(-18)
        else:
            print(f"{Style.DIM}{Fore.RED}Steering ANTICLOCKWISE because too close to the front wall{Style.RESET_ALL}")
            setSteeringAngle(18, importance = 0) #
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
                setSteeringAngle(18, importance = 0)
            elif 0 < rightSlope < 0.25 and direction == CLOCKWISE:# and leftSlope > 0:
                # Go as CLOCKWISE possible, because facing the wall
                print(Fore.RED + "Go as CLOCKWISE possible, because facing the wall" + Style.RESET_ALL)
                setSteeringAngle(-18, importance = 0)
            else:
                # Go as CLOCKWISE or ANTICLOCKWISE possible, because facing the wall
                print(Fore.RED + "Go as CLOCKWISE or ANTICLOCKWISE possible, because facing the wall" + Style.RESET_ALL)
                setSteeringAngle(-18 * direction, importance = 0)
        elif leftFlagIn and not(rightFlagIn):
            # Go a bit right
            print(Fore.RED + "Go a bit right" + Style.RESET_ALL)
            setSteeringAngle(-8)
        elif rightFlagIn and not(leftFlagIn):
            # Go a bit left
            print(Fore.RED + "Go a bit left" + Style.RESET_ALL)
            setSteeringAngle(9)#8
        elif not(leftFlagIn) and leftSlope > rightSlope * 1.2 and not(leftRect[0]+leftRect[1]<130 and rightRect[1] == 0 and direction != CLOCKWISE): #leftFlagOut or 
            setSteeringAngle(8)#7
            print("Shift a little left")
        else:
            # Go straight
            print(Style.DIM + Fore.GREEN + "Go straight" + Style.RESET_ALL)
            if direction == 0:
                setSteeringAngle(2)#0
            elif direction == CLOCKWISE:
                setSteeringAngle(1)
            elif direction == ANTICLOCKWISE:
                setSteeringAngle(2)###4
    elif lowerRightFlag and not(rightFlagIn) and not(leftFlagIn) and frontFlag2:
        setSteeringAngle(10)#9
        print(f"{Fore.RED}Moving left beacuse too close to the corner{Style.RESET_ALL}")
    elif lowerLeftFlag and not(rightFlagIn) and not(leftFlagIn)  and frontFlag2:
        setSteeringAngle(-9)
        print(f"{Fore.RED}Moving right beacuse too close to the corner{Style.RESET_ALL}")
    else:
        print(Style.BRIGHT + Fore.GREEN + "Everything fine! Going straight!" + Style.RESET_ALL)
        if direction == 0:
            setSteeringAngle(2)#0
        elif direction == CLOCKWISE:
            setSteeringAngle(1)
        elif direction == ANTICLOCKWISE:
            setSteeringAngle(2)##4
    
    # Return value says whether the prototype is far (= True) or close (= False) respect to the wall
    return not(closeToWall)