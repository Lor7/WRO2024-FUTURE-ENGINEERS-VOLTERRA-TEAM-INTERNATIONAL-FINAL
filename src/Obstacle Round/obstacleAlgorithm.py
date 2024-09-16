from obstacleRecognitionConstants import focalLength, realMeasure, GREENBLOCKID, REDBLOCKID, obstacleWidth, obstacleHeight
from obstacleRecognitionConstants import CLOCKWISE, ANTICLOCKWISE, FRAME_HEIGHT, FRAME_WIDTH, correctionAreaPercentageLower, correctionAreaPercentageLower2, correctionAreaPercentageMid, cccL, lwL, cccR, lwR
from math import atan, atan2
from actuators import setSteeringAngle, reverse, simpleReverse
from time import time

steeringAngle = None
direction = 0
HALF_FRAME_WIDTH = FRAME_WIDTH // 2
HALF_FRAME_HEIGHT = FRAME_HEIGHT // 2
state = None
timeLastLine = None
#print = lambda *x, **y: None

def setState(_state, _timeLastLine):
    global state, timeLastLine
    state = _state
    timeLastLine = _timeLastLine
def setDirection(_direction):
    global direction
    direction = _direction

def avoidObstacle(block, rightWall, virtualPoint, line, block2, coloredAreaMid, coloredAreaLower, coloredAreaLower2,leftWall): 
    print = lambda *x, **y: None
    slope, estimatedDistance, goReverse, getSpace, closeToWall = 0, obstacleHeight * focalLength / block[3], False, False, False
    flagMid, flagLower, flagLower2 = True if coloredAreaMid > correctionAreaPercentageMid else False, True if coloredAreaLower > correctionAreaPercentageLower else False, True if coloredAreaLower2 > correctionAreaPercentageLower2 else False     
    """
    print(f"Block: {block}")
    print(f"Line: {line}")
    print(f"Right wall: {rightWall}")
    print(f"Left wall: {leftWall}")
    print(f"Estimated distance: {estimatedDistance}")
    print(f"Virtual point: {virtualPoint}")
    print(f"Flag mid: {flagMid}, flag lower: {flagLower}, flag lower2: {flagLower2}")
    """
    #print(f"Estimated distance: {estimatedDistance}")
    if line[0] != line[2]:
        slope = abs( (line[1] - line[3]) / (line[0] - line[2]) )
        #print(f"Slope: {slope}")
    
    deltaX = HALF_FRAME_WIDTH - virtualPoint
    #print(f"Delta x: {deltaX}")
    deltaY = FRAME_HEIGHT - block[1] - block[3]
    #print(f"Delta y: {deltaY}")
    angle = atan2(deltaX, deltaY) * 57.3
    #print(f"Angle: {angle}")    
    
    steeringAngleCopy = steeringAngle = angle / 2.1
    #print(f"Steering angle: {steeringAngle}")
    
    #
    ###Case 6 in finalImageProcessing, filtro sui blocchi visti
    #
    
    if direction == CLOCKWISE or direction == 0:
        if block[4] == REDBLOCKID:
            
            
            #if slope != 0 and False:
            #    virtualPoint_y = block[1] + block[3]
            #    dX = virtualPoint - line[0]
            #    virtualLineY = line[1] + dX * slope
            #    if virtualPoint_y > virtualLineY:
            #        obstacleBehindTheWall = False
            #        #print(f"Virtual point NOT behind the wall: {virtualPoint_y}, line: {virtualLineY}, var: {obstacleBehindTheWall}")
            #    else:
            #        obstacleBehindTheWall = True
            #        #print(f"Virtual point behind the wall: {virtualLineY}, virtualPoint: {virtualPoint_y}, var: {obstacleBehindTheWall}")
            #else:
            #    obstacleBehindTheWall = False
            
            #Case 2, il resto dell'implementazione e
            #if obstacleBehindTheWall and (block[0] > 260 and 50 < estimatedDistance < 110) and (flagMid or flagLower or flagLower2) and time() - timeLastLine[0] > 6:
            #    if flagMid or block[0]+block[2]+rightWall[0] < 580:
            #        steeringAngle = 14
            #        print(f"Case 2, e.c.1  {steeringAngle}")
            #    else:
            #        steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] + block[2]*1.5, deltaY) * 57.3 / 2.1
            #        steeringAngle = max(steeringAngle, 9)
            #        print(f"Case 2, e.c.2  {steeringAngle}")
            
            #Case 12
            if block[0] < 130 and block[1]+block[3] > 300 and block[4] == REDBLOCKID and not(flagMid or flagLower or flagLower2) and (rightWall[2] == 0 or rightWall[0] > 300) and block2[0] != -1 and  block2[4] == REDBLOCKID and 40 < obstacleHeight * focalLength / block2[3] < 85 and block2[0] > 390:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block2[0] + block2[2] / 2, deltaY) * 57.3 / 2.1
                print(f"Case 12, 1: {steeringAngle}")
                
            
            #Case 3
            elif 0 < rightWall[0] < 81 and block[0]+block[2] > 340 and estimatedDistance > 50:                
                if 0 < slope < 1.6:
                    if flagLower2 and flagLower and flagMid:
                        steeringAngle = (atan(1.3) - atan(slope)) * 38.2#/ 2.1 * 57.3 * 1.4
                        print(f"Case 3 (using slope A): {steeringAngle}")
                    elif not(flagLower2) and flagLower:
                        steeringAngle = (atan(0.863) - atan(slope)) * 38.2#/ 2.1 * 57.3 * 1.4
                        print(f"Case 3 (using slope B): {steeringAngle}")
                elif leftWall[2] != 0 and rightWall[2] != 0:
                    steeringAngle = atan2(HALF_FRAME_WIDTH - ( leftWall[0] + leftWall[2] + (block[0] + block[2] + rightWall[0] - leftWall[2])/2 ), deltaY) / 2.1 * 57.3
                    print(f"Case 3 (using walls): {steeringAngle}")
            
            # Case 5
            elif flagMid and slope > 0.3 and (
                (flagLower and block[0]+block[2] > 100 and 0 < leftWall[2] < 60 and estimatedDistance < 48)
                or (estimatedDistance < 65 and 50 < leftWall[0]+leftWall[2] < 210 and 420 < block[0]+block[2]+rightWall[0] < 565 and slope > 0.5)) :
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2] - rightWall[0] / 2, deltaY) * 57.3 / 2.1
                print(f"Case #5: {steeringAngle}")
                
                
            # Case 4
            elif estimatedDistance < 29 and slope < 0.25 and block[0]+block[2] > 120 and rightWall[2] != 0:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2] - rightWall[0] / 2, deltaY) * 57.3 / 2.1
                print(f"Case #4, 1: {steeringAngle}")
            elif estimatedDistance < 27 and block[0]+block[2] > 110 and rightWall[2] == 0 and (slope == 0 or slope > 0.25):
                steeringAngle = min(steeringAngle, -17)
                print(f"Case #4, 2: {steeringAngle}")
            elif estimatedDistance < 35 and block[0]+block[2] > 120 and (block[0]+block[2]+rightWall[0] > 565) and 0.75 < slope < 1.8:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2] - rightWall[0] / 2, deltaY) * 57.3 / 2.1
                print(f"Case #4, 3: {steeringAngle}")
                
            # Case 2
            elif 39 < estimatedDistance < 75 and time() - timeLastLine[0] > 5 and slope != 0 and (
                (block[0]+block[2] > 280 and leftWall[2] == 0) or (block[0]+block[2] > 350 and leftWall[2] < 130)):
                if flagLower2 and flagLower and flagMid:
                    steeringAngle = 14
                    print(f"Case 2, #1: {steeringAngle}")
                elif flagLower2 and flagLower and not(flagMid):
                    steeringAngle = 11
                    print(f"Case 2, #2: {steeringAngle}")
                elif flagLower2 and not(flagLower or flagMid):
                    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] + block[2]/3, deltaY) * 57.3 / 2.1
                    print(f"Case 2, #3: {steeringAngle}")
                else:
                    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] + block[2]/3, deltaY) * 57.3 / 2.1
                    print(f"Case 2, #4: {steeringAngle}")
            elif (slope > 1.7 and estimatedDistance > 37) or not( True or
                flagLower or flagLower2 or flagMid or slope == 0 or (leftWall[0] + leftWall[2] > 130 or leftWall[2] < 1) or estimatedDistance < 41 or block[0] < 320):
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0], deltaY) * 57.3 / 2.1
                print(f"Case 2, #5: {steeringAngle}")
            elif estimatedDistance < 62 and 0 < slope < 0.47 and flagLower and flagLower2 and block[0] + block[2] < 335 and block[0]+block[2]+rightWall[0]<545:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0], deltaY) * 57.3 / 2.1 #+ block[2]
                print(f"Case 2, #6: {steeringAngle}")
            
            
            #Case 7
            elif 220 < rightWall[0] < 602 and flagLower2 and slope > 0.3 and leftWall[0]+leftWall[2]<100:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2] - rightWall[0] / 2, deltaY) * 57.3 / 2.1
                print(f"Case #7, 1: {steeringAngle}")
            elif rightWall[2] != 0 and block[0] > 130 and leftWall[0]+leftWall[2] < 175 and 390 < block[0]+block[2]+rightWall[0] < 610  and 19 < estimatedDistance < 97:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2] - rightWall[0] / 2, deltaY) * 57.3 / 2.1
                print(f"Case #7, 2: {steeringAngle}")
            
            #Case 10
            elif block2[0] != -1 and block[0] < 45 and block[0]+block[2] < 145 and (flagLower or flagLower2) and (
                (obstacleHeight * focalLength / block2[3] < 70)) and block2[4] == REDBLOCKID and block[0]+block[2]+rightWall[0] > 575:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block2[0] - block2[2]/2, FRAME_HEIGHT - block2[1] - block2[3]) * 57.3 / 2.1
                print(f"Case #10, 1: {steeringAngle}")
            
            #Case 8
            elif block[0] < 16 and block[2] < 90 and rightWall[2] == 0 and steeringAngle > 0 and block[1] + block[3] > 320:
                steeringAngle = 0
                print(f"Case #8, 1: {steeringAngle}")
            elif rightWall[2] == 0 and steeringAngle < 0 and block[1]+block[3] > 310 and block[0] < 30 and block[2] < 105:
                if block[0]+block[2] < 20:
                    steeringAngle = -3
                elif block[0]+block[2] < 75:
                    steeringAngle = 0
                elif block[0]+block[2] < 90:
                    steeringAngle = 7
                elif block[0]+block[2] < 120:
                    steeringAngle = 12
                else:
                    steeringAngle = 15
                print(f"Case #8, 2: {steeringAngle}")

        elif block[4] == GREENBLOCKID:
            
            #Case 9
            if flagLower and flagMid and slope < 0.35 and leftWall[2] == 0 and steeringAngle > 5 and block[0] + block[2] > 625 and block[0] > 500 and block[1] + block[3] > 320:
                steeringAngle = 5
                print(f"Case #9, 1: {steeringAngle}")
            elif block[0] > 544 and leftWall[2] == 0 and steeringAngle < -5 and block[1] + block[3] > 320:
                steeringAngle = -5
                print(f"Case #9, 2: {steeringAngle}")
            elif block[0] > 530 and block[0]+block[2] > 630 and steeringAngle < 0 and leftWall[2] == 0 and block[1]+block[3] > 330:
                steeringAngle = 12
                print(f"Case #9, 3: {steeringAngle}")
                
                
                  
                  
                  
                  
    if direction == ANTICLOCKWISE or direction == 0:
        
        if block[4] == GREENBLOCKID:
            
            if slope != 0:
                virtualPoint_y = block[1] + block[3]
                dX = virtualPoint - line[0]
                virtualLineY = line[1] - dX * slope
                if virtualPoint_y > virtualLineY:
                    obstacleBehindTheWall = False
                    #print(f"Virtual point NOT behind the wall: {virtualPoint_y}, line: {virtualLineY}, var: {obstacleBehindTheWall}")
                else:
                    obstacleBehindTheWall = True
                    #print(f"Virtual point behind the wall: {virtualLineY}, virtualPoint: {virtualPoint_y}, var: {obstacleBehindTheWall}")
            else:
                obstacleBehindTheWall = False
            
            #Case 2, il resto dell'implementazionee
            if obstacleBehindTheWall and (block[0] < 380 and 50 < estimatedDistance < 110) and (flagMid or flagLower or flagLower2) and time() - timeLastLine[0] > 6:
                if flagMid or leftWall[0]+leftWall[2] > 120:
                    steeringAngle = -14
                    print(f"Case 2, e.c.1  {steeringAngle}")
                else:
                    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2.5, deltaY) * 57.3 / 2.1
                    steeringAngle = min(steeringAngle, -9)
                    print(f"Case 2, e.c.2  {steeringAngle}")
            
            #Case 3
            # questo 'elif' era un 'if' prima di 04-11-2023
            elif block[0] > block[0]-leftWall[0]-leftWall[2] < 81 and block[0] < 300 and estimatedDistance > 50:                
                if 0 < slope < 1.82:
                    # Sia al caso A che al caso B potrebbe essere il caso di aggiungere (-2). Inoltre va correctta
                    # slope A = 1.3; slope B = 0.863
                    if flagLower2 and flagLower and flagMid:
                        steeringAngle = -1 + (atan(1.15) - atan(slope)) * 38.2#/ 2.1 * 57.3 * 1.4
                        print(f"Case 3 (using slope A): {steeringAngle}")
                    elif flagLower:
                        steeringAngle = (atan(0.76) - atan(slope)) * 38.2#/ 2.1 * 57.3 * 1.4
                        if obstacleBehindTheWall:
                            steeringAngle -= 2
                        print(f"Case 3 (using slope B): {steeringAngle}")
                elif slope > 1.8:
                    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2 - leftWall[0] - leftWall[2], deltaY) * 57.3 / 2.1
                    print(f"Case #3, 3: {steeringAngle}")
                elif leftWall[2] != 0 and rightWall[2] != 0:
                    steeringAngle = atan2(HALF_FRAME_WIDTH - ( leftWall[0] + leftWall[2] + (block[0] + block[2] + rightWall[0] - leftWall[2])/2 ), deltaY) / 2.1 * 57.3
                    print(f"Case 3 (using walls): {steeringAngle}")

            # Case 5
            # Implementation might not be needed
            
            
            # Case 4
            # First and Last possibilities are "translated" but disabled manually, as for 'Case 5' they might not be needed
            elif False and estimatedDistance < 29 and slope < 0.25 and block[0] < 520 and leftWall[2] != 0:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] + block[2] - leftWall[0] - leftWall[2], deltaY) * 57.3 / 2.1
                print(f"Case #4, 1: {steeringAngle}")
            elif estimatedDistance < 27 and block[0] < 520 and leftWall[2] == 0 and (slope == 0 or slope > 0.25):
                steeringAngle = max(steeringAngle, 17)
                print(f"Case #4, 2: {steeringAngle}")
            elif False and estimatedDistance < 35 and block[0] < 520 and (leftWall[0]+leftWall[2] < 75) and 0.75 < slope < 1.8:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] + block[2] - leftWall[0] - leftWall[2], deltaY) * 57.3 / 2.1
                print(f"Case #4, 3: {steeringAngle}")

            # Case 2
            elif 39 < estimatedDistance < 85 and time() - timeLastLine[0] > 5 and slope != 0 and (block[0] - leftWall[0] - leftWall[2] < 250) and (
                (block[0] < 390 and (rightWall[2] == 0 or rightWall[0] > 150)) or (block[0] < 325 and rightWall[2] < 155)):
                if flagLower2 and flagLower and flagMid:
                    steeringAngle = -14
                    print(f"Case 2, #1: {steeringAngle}")
                elif flagLower2 and flagLower and not(flagMid) and not(270 < block[0] < 330 and estimatedDistance < 65 and 455 < block[0]+block[2]+rightWall[0] < 570 and slope > 0.8 and leftWall[0]+leftWall[2] > 70):
                    steeringAngle = -11
                    print(f"Case 2, #2: {steeringAngle}")
                elif flagLower2 and not(flagLower or flagMid):
                    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2, deltaY) * 57.3 / 2.1
                    print(f"Case 2, #3: {steeringAngle}")
                #else:
                #    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2, deltaY) * 57.3 / 2.1
                #    print(f"Case 2, #4: {steeringAngle}")
            elif (slope > 1.7 and estimatedDistance > 37 and (flagLower or flagLower2 or flagMid or leftWall[0]+leftWall[2] > 200)) or (slope > 1.7 and estimatedDistance > 75 and rightWall[2] != 0):
                #or ( True or flagLower or flagLower2 or flagMid or slope == 0 or (block[0]+block[2]+rightWall[0] < 510 or rightWall[2] == 0) or estimatedDistance < 41 or block[0] > 290):
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2, deltaY) * 57.3 / 2.1
                print(f"Case 2, #5: {steeringAngle}")
            elif estimatedDistance < 62 and 0 < slope < 0.47 and flagLower and flagLower2 and block[0] > 315 and leftWall[0]+leftWall[2] > 90:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2, deltaY) * 57.3 / 2.1
                print(f"Case 2, #6: {steeringAngle}")
            #elif estimatedDistance < 46 and slope > 0.6 and flagLower2 and block[0] > 320 and leftWall[0]+leftWall[2] < 90 and block[0]+block[2]+rightWall[0] > 560:
            #    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2, deltaY) * 57.3 / 2.1
            #    print(f"Case 2, #7: {steeringAngle}")
            #elif 40 < estimatedDistance < 100 and block[0] < 320 and leftWall[0]+leftWall[2] < 170 and rightWall[2] != 0:
            #    steeringAngle = atan2(HALF_FRAME_WIDTH - block[0] - block[2]*2, deltaY) * 57.3 / 2.1
            #    print(f"Case 2, #8: {steeringAngle}")

            
            # Case 7
            # As well as 'Case 5' Implementation might not be needed

            #Case 10
            elif block2[0] != -1 and block[0]+block[2] > 595 and block[0] > 495 and (flagLower or flagLower2) and (
                (obstacleHeight * focalLength / block2[3] < 70)) and block2[4] == GREENBLOCKID and leftWall[0]+leftWall[2] < 65:
                steeringAngle = atan2(HALF_FRAME_WIDTH - block2[0] - block2[2]/2, FRAME_HEIGHT - block2[1] - block2[3]) * 57.3 / 2.1
                print(f"Case #10, 1: {steeringAngle}")

            #Case 8
            elif block[0]+block[2] > 624 and block[0] > 550 and leftWall[2] == 0 and steeringAngle < 0 and block[1] + block[3] > 320:
                steeringAngle = 0
                print(f"Case #8, 1: {steeringAngle}")
            elif block[0] > 520 and leftWall[2] == 0 and steeringAngle < 0 and block[1] + block[3] > 260:
                steeringAngle = 0
                print(f"Case #8, 2: {steeringAngle}")
                

        elif block[4] == REDBLOCKID:
            #Case 11
            if estimatedDistance > 80 and block[0] < 400 and slope == 0 and rightWall[0] < 100 and block[0]-leftWall[0]-leftWall[2] < 100:
                steeringAngle = -13
                print(f"Case #11, 1: {steeringAngle}")
            #Case 9
            elif rightWall[2] == 0 and steeringAngle < 0 and block[1]+block[3] > 300 and block[0] < 30 and block[2] < 105:
                if block[0]+block[2] < 20:
                    steeringAngle = -3
                elif block[0]+block[2] < 75:
                    steeringAngle = 0
                elif block[0]+block[2] < 90:
                    steeringAngle = 7
                elif block[0]+block[2] < 120:
                    steeringAngle = 12
                else:
                    steeringAngle = 15
                print(f"Case #9, 1: {steeringAngle}")
            elif flagLower and flagMid and slope < 0.35 and rightWall[2] == 0 and steeringAngle < -5 and block[0] < 15 and block[0]+block[2] < 140 and block[1] + block[3] > 320:
                steeringAngle = -5
                print(f"Case #9, 2: {steeringAngle}")
            elif block[0]+block[2] < 96 and rightWall[2] == 0 and steeringAngle > 4 and block[1] + block[3] > 320:
                steeringAngle = 4
                print(f"Case #9, 3: {steeringAngle}")
            elif block[0]+block[2] < 120 and virtualPoint < 320 and rightWall[2] == 0:
                 steeringAngle = 0
                 print(f"Case #9, 4: {steeringAngle}")
    
    
    if steeringAngleCopy == steeringAngle:
        print(f"Normal steering: {steeringAngle}")
    setSteeringAngle(steeringAngle, importance = 2)
    
    
    print()

    
    return not(closeToWall)