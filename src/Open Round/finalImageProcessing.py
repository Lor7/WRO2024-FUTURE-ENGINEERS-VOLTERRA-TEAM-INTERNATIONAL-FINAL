from multiprocessing import Process, Array, Lock
from threading import Thread
from time import sleep
import cv2
from wallRecognitionConstants import *
#from brain import makeDecision

class ImageProcessing():

    def __init__(self, shared_values, lock, showFrameFlag = False, drawDebug = False):
        self.lock = lock
        self.shared_values = shared_values if shared_values != None else [i for i in range(0)]
        self.INTERRUPT = False
        self.frame = None
        self.capture = None
        self.showFrameFlag = showFrameFlag
        self.image = None
        self.frame2 = None
        self.NEWFRAME = False
        self.PROCESSED = False
        self.direction = 0
        self.drawDebug = drawDebug
    def setDirection(self, direction):
        global wlcC, cccL, cccL2, cccL3, lwL, cccR, cccR2, cccR3, lwR, CLOCKWISE, ANTICLOCKWISE
        self.direction = direction
        if direction == CLOCKWISE:
            wlcC = (wlcC[0], wlcC[1], wlcC[2] - 15, wlcC[3] + 25)
            cccL = (cccL[0] - 9, cccL[1], cccL[2] - 7, cccL[3] + 3)# -5, ., ., .,
            cccL2 = (cccL2[0] - 9, cccL2[1], cccL2[2] - 7, cccL2[3] + 3)
            cccL3 = (cccL3[0] - 9, cccL3[1], cccL3[2] - 7, cccL3[3] + 3)
            lwL = (lwL[0], lwL[1], lwL[2] - 7, lwL[3] + 3)
            cccR = (cccR[0] - 7, cccR[1], cccR[2] + 7, cccR[3])
            cccR2 = (cccR2[0] - 7, cccR2[1], cccR2[2] + 7, cccR2[3])
            cccR3 = (cccR3[0] - 7, cccR3[1], cccR3[2] + 7, cccR3[3])
            lwR = (lwR[0] - 7, lwR[1], lwR[2] + 7, lwR[3])
        elif direction == ANTICLOCKWISE:
            cccL = (cccL[0] - 35, cccL[1], cccL[2], cccL[3])
            cccL2 = (cccL2[0] - 35, cccL2[1], cccL2[2], cccL2[3])
            cccL3 = (cccL3[0] - 35, cccL3[1], cccL3[2], cccL3[3])
            lwL = (lwL[0], lwL[1], lwL[2], lwL[3])
            cccR = (cccR[0] - 15, cccR[1], cccR[2] + 15, cccR[3])
            cccR2 = (cccR2[0] - 15, cccR2[1], cccR2[2] + 15, cccR2[3])
            cccR3 = (cccR3[0] - 15, cccR3[1], cccR3[2] + 15, cccR3[3])
            lwR = (lwR[0] - 20, lwR[1], lwR[2], lwR[3] + 20)
    def getFrame(self):
        points = array([[240, 960], [360, 780], [1020, 760], [1220, 830], [1260, 960]])
        self.capture = cv2.VideoCapture(0)
        sleep(0.5)
        # La risoluzione si può abbassare ma si perde in qualità
        self.capture.set(3, 640)
        self.capture.set(4, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 60)
        sleep(3)
        #while not(self.capture.isOpened):
        #    sleep(0.1)
        while not(self.INTERRUPT):
            _, frame2 = self.capture.read()
            if _:
                frame2 = cv2.flip(frame2.copy(), -1)
                self.image = cv2.fillPoly(frame2, pts = [points], color = (255, 255, 255))
                self.NEWFRAME = True
                sleep(0.02)
        self.capture.release()
    def showFrame(self):
        try:
            while self.frame == None:
                sleep(0.1)
        except ValueError:
            pass
        while not(self.INTERRUPT):
            sleep(0.1)
            if self.PROCESSED:
                self.PROCESSED = False
                cv2.imshow("Frame", self.frame)
                if cv2.waitKey(1) == 0xFF & ord('q'):
                    cv2.destroyAllWindows()
                    self.capture.release()
    
    def processFrame(self):
        from colorama import Fore, Back, Style
        from os import system
        
        walls, line = [], []
        line = [0, 0, 0, 0]
        frame, image, hsv = None, None, None
        coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn = -1, -1, -1
        coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn, coloredAreaFront = -1, -1, -1, -1
        coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2 = -1, -1, -1 
        leftRect, rightRect, leftLine, rightLine = (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0)
        wallSortingFunc = lambda w: w[2] + w[0]

        def detectWallRect(hsv, cuts):
            blur = cv2.GaussianBlur(hsv[cuts[2] : cuts[2] + cuts[3], cuts[0]: cuts[0] + cuts[1]].copy(), (13, 13), cv2.BORDER_REFLECT_101)
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            result = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.BORDER_DEFAULT)
            rcnt, hierarchy = result if len(result) == 2 else result[1:3]
            walls = []
            for i in rcnt:
                x, y, w, h = cv2.boundingRect(i)
                if (w > 35 and h > 5):#20, 10
                    area = cv2.contourArea(i)
                    if (area > w * h * 0.36):
                        walls.append([x, w, y, h])
            walls.sort(reverse = False, key = wallSortingFunc)   
            return walls[0] if walls != [] else []
        def detectWallLines(hsv, wlc):
            blur = cv2.GaussianBlur(hsv[wlc[2] : wlc[2] + wlc[3], wlc[0]: wlc[0] + wlc[1]].copy(), (13, 13), cv2.BORDER_REFLECT_101)
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            canny = cv2.Canny(maskb, 50, 140)
            lines = cv2.HoughLinesP(canny, 1, pi/180, 40, minLineLength = 20, maxLineGap = 45)#40, 40, 90
            lower = [0, 0, 0, 0]
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if y1 > lower[1]:
                        lower[0] = x1
                        lower[1] = y1
                        lower[2] = x2
                        lower[3] = y2
            return lower
        def wallInfo(hsv, ccc):
            blur = cv2.GaussianBlur(hsv[ccc[2] : ccc[2] + ccc[3], ccc[0]: ccc[0] + ccc[1]].copy(), (13, 13), cv2.BORDER_REFLECT_101)
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            result = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.BORDER_DEFAULT)
            cnt, hierarchy = result if len(result) == 2 else result[1:3]
            coloredArea = 0
            for i in cnt: 
                coloredArea += cv2.contourArea(i) 
                cv2.fillPoly(frame[ccc[2]:FRAME_HEIGHT, ccc[0]:FRAME_WIDTH] , pts = [i], color = (255, 255, 255))
            return coloredArea
        
        while not(self.INTERRUPT):
            while not(self.NEWFRAME):
                sleep(0.0001)
            self.NEWFRAME = False
            walls = []

            #frame = cv2.resize(self.image.copy(), (640, 480), interpolation = cv2.INTER_NEAREST)
            frame = self.image.copy()
            hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)        
        
            frame = cv2.rectangle(frame, (cccL[0], cccL[2]), (cccL[0] + cccL[1], cccL[2] + cccL[3]), (30, 210, 80), 3)
            frame = cv2.rectangle(frame, (cccR[0], cccR[2]), (cccR[0] + cccR[1], cccR[2] + cccR[3]), (30, 210, 80), 3)                
            frame = cv2.rectangle(frame, (cccL2[0], cccL2[2]), (cccL2[0] + cccL2[1], cccL2[2] + cccL2[3]), (30, 210, 80), 3)
            frame = cv2.rectangle(frame, (cccR2[0], cccR2[2]), (cccR2[0] + cccR2[1], cccR2[2] + cccR2[3]), (30, 210, 80), 3)
            frame = cv2.rectangle(frame, (cccL3[0], cccL3[2]), (cccL3[0] + cccL3[1], cccL3[2] + cccL3[3]), (30, 210, 80), 3)
            frame = cv2.rectangle(frame, (cccR3[0], cccR3[2]), (cccR3[0] + cccR3[1], cccR3[2] + cccR3[3]), (30, 210, 80), 3)
            frame = cv2.rectangle(frame, (mpcL[0], mpcL[2]), (mpcL[0] + mpcL[1], mpcL[2] + mpcL[3]), (120, 190, 25), 1)
            frame = cv2.rectangle(frame, (mpcR[0], mpcR[2]), (mpcR[0] + mpcR[1], mpcR[2] + mpcR[3]), (120, 190, 25), 1)
            frame = cv2.rectangle(frame, (wlcL[0], wlcL[2]), (wlcL[0] + wlcL[1], wlcL[2] + wlcL[3]), (150, 30, 190), 1)
            frame = cv2.rectangle(frame, (wlcR[0], wlcR[2]), (wlcR[0] + wlcR[1], wlcR[2] + wlcR[3]), (150, 30, 190), 1)
            frame = cv2.rectangle(frame, (wlcC[0], wlcC[2]), (wlcC[0] + wlcC[1], wlcC[2] + wlcC[3]), (150, 30, 190), 1)
            frame = cv2.rectangle(frame, (cccf[0], cccf[2]), (cccf[0] + cccf[1], cccf[2] + cccf[3]), (25, 130, 190), 1)
            frame = cv2.rectangle(frame, (cccf2[0], cccf2[2]), (cccf2[0] + cccf2[1], cccf2[2] + cccf2[3]), (100, 130, 190), 3)
            frame = cv2.rectangle(frame, (lwL[0], lwL[2]), (lwL[0] + lwL[1], lwL[2] + lwL[3]), (15, 100, 215), 3)
            frame = cv2.rectangle(frame, (lwR[0], lwR[2]), (lwR[0] + lwR[1], lwR[2] + lwR[3]), (15, 100, 215), 3)
            
            centralLine = detectWallLines(hsv, wlcC)
            
            # Getting wall areas
            coloredAreaLeftMid = int(wallInfo(hsv, cccL))
            coloredAreaRightMid = int(wallInfo(hsv, cccR))
            coloredAreaLeftIn = int(wallInfo(hsv, cccL2))
            coloredAreaRightIn = int(wallInfo(hsv, cccR2))
            coloredAreaLeftOut = int(wallInfo(hsv, cccL3))
            coloredAreaRightOut = int(wallInfo(hsv, cccR3))
            coloredAreaFront = int(wallInfo(hsv, cccf))
            coloredAreaFront2 = int(wallInfo(hsv, cccf2))
            coloredAreaLowerLeft = int(wallInfo(hsv, lwL))
            coloredAreaLowerRight = int(wallInfo(hsv, lwR))

            # Getting wall rects
            leftRect = detectWallRect(hsv, mpcL)
            rightRect = detectWallRect(hsv, mpcR)
            if len(leftRect) > 0:
                leftRect = (leftRect[0], leftRect[1], leftRect[2] + mpcL[2], leftRect[3])
                frame = cv2.rectangle(frame, (leftRect[0], leftRect[2]), (leftRect[0] + leftRect[1], leftRect[2] + leftRect[3]), (150, 120, 25), 3)
            if len(rightRect) > 0:
                rightRect = (rightRect[0] + mpcL[1], rightRect[1], rightRect[2] + mpcR[2], rightRect[3])
                frame = cv2.rectangle(frame, (rightRect[0], rightRect[2]), (rightRect[0] + rightRect[1], rightRect[2] + rightRect[3]), (150, 120, 25), 3)
            #print(leftRect, rightRect)
            
            # Getting wall lines
            leftLine = detectWallLines(hsv, wlcL)
            rightLine = detectWallLines(hsv, wlcR)
            if len(leftLine) > 0:
                leftLine = (leftLine[0], leftLine[1] + wlcL[2], leftLine[2], leftLine[3] + wlcL[2])
                frame = cv2.line(frame, (leftLine[0], leftLine[1]), (leftLine[2], leftLine[3]), (180, 10, 180), 3)
            if len(rightLine) > 0:
                rightLine = (rightLine[0] + wlcR[0], rightLine[1] + wlcR[2], rightLine[2] + wlcR[0], rightLine[3] + wlcR[2])
                frame = cv2.line(frame, (rightLine[0], rightLine[1]), (rightLine[2], rightLine[3]), (180, 10, 180), 3)
            if len(centralLine) > 0:
                centralLine = (centralLine[0] + wlcC[0], centralLine[1] + wlcC[2], centralLine[2] + wlcC[0], centralLine[3] + wlcC[2])
                frame = cv2.line(frame, (centralLine[0], centralLine[1]), (centralLine[2], centralLine[3]), (180, 10, 180), 3)

            if leftRect == []:
                leftRect = [0, 0, 0, 0]
            if rightRect == []:
                rightRect = [0, 0, 0, 0]

            #makeDecision(coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn, 
            #coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn, coloredAreaFront,
            #leftRect, rightRect, leftLine, rightLine,
            #coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2)


    def run(self):
        getFrameThread = Thread(target = self.getFrame)        
        processFrameThread = Thread(target = self.processFrame)
        getFrameThread.start()
        processFrameThread.start()
        if self.showFrameFlag:
            showFrameThread = Thread(target = self.showFrame)
            showFrameThread.start()
    def stop(self):
        try:
            self.INTERRUPT = True
            self.capture.release()
            cv2.destroyAllWindows()
        except:
            pass
        
if __name__ == '__main__':
    lock = Lock()
    shared_values = Array('d', range(31))
    imageProcessing = ImageProcessing(shared_values, lock, True, False)
    imageProcessing_process = Process(target = imageProcessing.run(), daemon = True)
    imageProcessing_process.start()

    sleep(1)
    while True:
        try:
            ##print(f"Shared values: {list(shared_values)}")
            ##print(imageProcessing.frame)
            sleep(1)
        except KeyboardInterrupt:
            break
    imageProcessing.stop()
    imageProcessing_process.terminate()
    exit()