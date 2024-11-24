from threading import Thread, Lock
from time import sleep, time, perf_counter
import cv2
from obstacleRecognitionConstants import *
import numpy as np
from park import handleParking

import zmq
import subprocess
import os
import sys
import glob
import importlib.util

# Set flag for using TPU (Tensor Processing Unit)
use_TPU = True

# Define paths for the model and label files
CWD_PATH = "/home/pi/Desktop/modellowro2024brescia/"
# CWD_PATH = "/home/pi/Desktop/ultimo_efficientnet/"
MODEL_NAME = 'custom_model_lite'
GRAPH_NAME = 'edgetpu.tflite'
LABELMAP_NAME = 'labelmap.txt'
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

# Load label names from the label map file
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]
# Remove the first label if it is '???'
if labels[0] == '???':
    del labels[0]

# Check if tflite_runtime is available and import the appropriate module
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# Initialize the TFLite interpreter with TPU delegate if available
interpreter = Interpreter(model_path=PATH_TO_CKPT,
                            experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
interpreter.allocate_tensors()

# Get input and output details from the model
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Extract model input dimensions
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

# Check if the model uses floating-point numbers
floating_model = (input_details[0]['dtype'] == np.float32)

# Set mean and standard deviation for input normalization
input_mean = 127.5
input_std = 127.5

# Get the name of the output tensor
outname = output_details[0]['name']

# Define indices for the output tensors: bounding boxes, class labels, and scores
boxes_idx, classes_idx, scores_idx = 1, 3, 0

# Set the minimum confidence threshold for detecting objects
min_conf_threshold = 0.3



class ImageProcessing():
    def __init__(self, shared_values, lock, showFrameFlag=False, saveFrameFlag=False, drawDebug=False):
        # Initialize class variables
        self.lock = lock
        self.shared_values = shared_values if shared_values is not None else [i for i in range(0)]
        self.INTERRUPT = False  # Flag to stop the processing
        self.frame = None  # Stores the current frame
        self.capture = None  # Video capture object
        self.showFrameFlag = showFrameFlag  # Flag to show frames
        self.image = None  # Processed image
        self.frame2 = None  # Temporary frame for processing
        self.NEWFRAME = False  # Flag to indicate if a new frame is available
        self.PROCESSED = False  # Flag to indicate if the frame has been processed
        self.saveFrameFlag = saveFrameFlag  # Flag to save frames
        self.TOBESAVED = False  # Flag to mark if frame should be saved
        self.processingTimeList = []  # List to track processing times
        self.direction = 0  # Direction for image processing
        self.drawDebug = drawDebug  # Flag to draw debug information
        self.lap = 0  # Lap counter or other usage
        self.timeAllSameColorPark = 0  # Timer for parking color detection
        command = '''python3.11 "/home/pi/Desktop/cameraFeed.py"'''
        self.camera_feed_process = subprocess.Popen(command, shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    def setDirection(self, direction):
        # Update direction-specific parameters
        global cccL, cccL2, cccL3, lwL, cccR, cccR2, cccR3, lwR, CLOCKWISE, ANTICLOCKWISE, closerBlockFilter, HALF_FRAME_WIDTH, wlcC
        
        if self.direction != 0:
            # Adjust the region of interest based on the previous direction
            if direction == CLOCKWISE:
                wlcC = (wlcC[0], wlcC[1], wlcC[2] + 25, wlcC[3] - 25)
                cccL = (cccL[0] + 5, cccL[1], cccL[2] + 10, cccL[3] - 15)
                cccL2 = (cccL2[0] + 5, cccL2[1], cccL2[2] + 10, cccL2[3] - 15)
                cccL3 = (cccL3[0] + 5, cccL3[1], cccL3[2] + 10, cccL3[3] - 15)
                lwL = (lwL[0], lwL[1], lwL[2] + 10, lwL[3] - 15)
            elif direction == ANTICLOCKWISE:
                cccL = (cccL[0] + 10, cccL[1], cccL[2], cccL[3])
                cccL2 = (cccL2[0] + 10, cccL2[1], cccL2[2], cccL2[3])
                cccL3 = (cccL3[0] + 10, cccL3[1], cccL3[2], cccL3[3])
                lwL = (lwL[0], lwL[1], lwL[2], lwL[3])
                cccR = (cccR[0] + 5, cccR[1], cccR[2] - 25, cccR[3])
                cccR2 = (cccR2[0] + 5, cccR2[1], cccR2[2] - 25, cccR2[3])
                cccR3 = (cccR3[0] + 5, cccR3[1], cccR3[2] - 25, cccR3[3])
                lwR = (lwR[0] + 5, lwR[1], lwR[2] - 15, lwR[3])

        self.direction = direction  # Update direction

        # Adjust regions of interest based on the new direction
        if direction == CLOCKWISE:
            wlcC = (wlcC[0], wlcC[1], wlcC[2] - 25, wlcC[3] + 25)
            cccL = (cccL[0] - 5, cccL[1], cccL[2] - 10, cccL[3] + 15)
            cccL2 = (cccL2[0] - 5, cccL2[1], cccL2[2] - 10, cccL2[3] + 15)
            cccL3 = (cccL3[0] - 5, cccL3[1], cccL3[2] - 10, cccL3[3] + 15)
            lwL = (lwL[0], lwL[1], lwL[2] - 10, lwL[3] + 15)
            closerBlockFilter = lambda l: l[1] + l[3] - abs(((305 - l[0] - l[2]/2) / HALF_FRAME_WIDTH)**2 * 50)
        elif direction == ANTICLOCKWISE:
            cccL = (cccL[0] - 10, cccL[1], cccL[2], cccL[3])
            cccL2 = (cccL2[0] - 10, cccL2[1], cccL2[2], cccL2[3])
            cccL3 = (cccL3[0] - 10, cccL3[1], cccL3[2], cccL3[3])
            lwL = (lwL[0], lwL[1], lwL[2], lwL[3])
            cccR = (cccR[0] - 5, cccR[1], cccR[2] + 25, cccR[3])
            cccR2 = (cccR2[0] - 5, cccR2[1], cccR2[2] + 25, cccR2[3])
            cccR3 = (cccR3[0] - 5, cccR3[1], cccR3[2] + 25, cccR3[3])
            lwR = (lwR[0] - 5, lwR[1], lwR[2] + 15, lwR[3])
            closerBlockFilter = lambda l: l[1] + l[3] - abs(((335 - l[0] - l[2]/2) / HALF_FRAME_WIDTH)**2 * 50)

    def getFrame(self):
        from colorama import Style, Fore
        # Define points for drawing polygons on the frame
        points = array([[125, 960], [170, 915], [330, 780], [1020, 760], [1245, 825], [1280, 960]]) // 2
        points2 = array([[0, 0], [0, 190], [0, 335], [160, 280], [320, 245], [650, 190], [750, 190], [950, 225], [1100, 245], [1280, 290], [1280, 0]]) // 2

        # Initialize video capture from camera
        self.capture = cv2.VideoCapture(0)
        sleep(0.5)  # Allow time for camera to initialize
        self.capture.set(3, 1280)  # Set frame width
        self.capture.set(4, 960)  # Set frame height
        self.capture.set(cv2.CAP_PROP_FPS, 30)  # Set frame rate
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Set buffer size
        sleep(3)  # Allow time for camera settings to take effect

        # Wait until the capture is opened
        while not self.capture.isOpened():
            sleep(0.1)

        # Main loop to capture and process frames
        while not self.INTERRUPT:
            try:
                while not self.INTERRUPT:
                    sleep(0.04)  # Brief sleep to regulate frame capture rate
                    start = perf_counter()  # Start timing the frame processing
                    _, frame2 = self.capture.read()  # Read a frame from the camera
                    if _:
                        # Resize and flip the frame, then apply a polygon mask
                        frame2 = cv2.flip(cv2.resize(frame2, (640, 480), cv2.INTER_LINEAR), -1)
                        self.image = cv2.fillPoly(frame2, pts=[points, points2], color=(255, 255, 255))
                        self.NEWFRAME = True  # Indicate a new frame has been processed
                        # Optionally print processing time
                        # print(f"{Style.BRIGHT}{Fore.GREEN}Processing time: {(perf_counter() - start) * 1000}ms{Style.RESET_ALL}")
            except Exception as e:
                print(f"getFrame error: {e}")
        self.capture.release()  # Release the video capture when done

    def showFrame(self):
        try:
            # Wait until the frame is available
            while self.frame is None:
                sleep(0.1)
        except ValueError:
            pass

        # Main loop to display frames
        while not self.INTERRUPT:
            sleep(0.05)  # Brief sleep to regulate frame display rate
            if self.PROCESSED:
                self.PROCESSED = False  # Reset processed flag
                cv2.imshow("Frame", self.frame.copy())  # Display the frame
                # Check for user input to quit
                if cv2.waitKey(1) == 0xFF & ord('q'):
                    self.INTERRUPT = True
                    sleep(0.25)
                    cv2.destroyAllWindows()  # Close all OpenCV windows
                    self.capture.release()  # Release the video capture
    
    def processFrame(self):
        from numpy import clip, zeros_like, float32, float16, mean, power
        from colorama import Fore, Back, Style
        from os import system
        global closerBlockFilter
        HALF_FRAME_WIDTH = FRAME_WIDTH // 2
        HALF_FRAME_HEIGHT = FRAME_HEIGHT // 2
        
        # Initialize variables
        virtualPoint = None
        walls, blocks, line = [], [], []
        line = [0, 0, 0, 0]
        frame, image, hsv = None, None, None
        coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn = -1, -1, -1
        coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn, coloredAreaFront = -1, -1, -1, -1
        coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2 = -1, -1, -1 
        leftRect, rightRect, leftLine, rightLine = (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0)
        wallSortingFunc = lambda w: w[2] + w[0]
        
        # Define block filter for closer objects
        closerBlockFilter = lambda l: l[1] + l[3] - abs(((305 - l[0] - l[2]/2) / HALF_FRAME_WIDTH)**2 * 50)
        
        # Function to enhance red and green colors in the frame
        def static_enhance_red_and_green2(frame):
            b, g, r = cv2.split(frame)
            r_enhanced = cv2.addWeighted(r, 1.5, np.zeros_like(r), 0, 0)  # Enhance red channel
            g_enhanced = cv2.addWeighted(g, 1.5, np.zeros_like(g), 0, 0)  # Enhance green channel
            enhanced_frame = cv2.merge([b, g_enhanced, r_enhanced])
            enhanced_frame = cv2.convertScaleAbs(enhanced_frame, alpha=1.5, beta=0)  # Adjust brightness and contrast
            return enhanced_frame

        # Function to detect obstacles in a mask
        def detectObstacle(mask, pillarId):
            result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.BORDER_DEFAULT)
            rcnt, hierarchy = result if len(result) == 2 else result[1:3]
            blocks = []
            for i in rcnt:
                x, y, w, h = cv2.boundingRect(i)
                if (14 < w < 370 and (h / w > 1.1) and (y > 120 or (y + h > 137 and y > 110))) and not(
                    pillarId == REDBLOCKID and 12 < x < 632 and 180 < y < 430 and w < 66) and not(
                    self.direction == ANTICLOCKWISE and ((145 < y < 185 and w < 32 and 370 < x < 565) or 
                    (w < 60 and h > 150 and x > 450 and h/w > 3.5))) and not(
                    (self.direction == CLOCKWISE or self.direction == 0) and x < 30 and w < 60 and 150 < y < 200 and h < 60):

                    if (cv2.contourArea(i) >= w * h * 0.3):
                        blocks.append([x, y, w, h, pillarId])
            
            # Merge overlapping blocks
            ind, i = len(blocks) - 1, 0
            while i < ind:
                if blocks[i][4] == blocks[i + 1][4] and (
                    (blocks[i][0] -20 < blocks[i + 1][0] < blocks[i][0] + blocks[i][2] +20) or
                    (blocks[i][0] -20 < blocks[i + 1][0] + blocks[i + 1][2] < blocks[i][0] + blocks[i][2] +20)) and (
                    (blocks[i][1] -20 < blocks[i + 1][1] < blocks[i][1] + blocks[i][3] +20) or
                    (blocks[i][1] -20 < blocks[i + 1][1] + blocks[i + 1][3] < blocks[i][1] + blocks[i][3] +20)):
                    x = min(blocks[i][0], blocks[i + 1][0])
                    y = min(blocks[i][1], blocks[i + 1][1])
                    x2 = max(blocks[i][0] + blocks[i][2], blocks[i + 1][0] + blocks[i + 1][2])
                    y2 = max(blocks[i][1] + blocks[i][3], blocks[i + 1][1] + blocks[i + 1][3])
                    blocks[i][0], blocks[i][1], blocks[i][2], blocks[i][3] = x, y, x2 - x, y2 - y
                    blocks.pop(i + 1)
                    ind -= 1
                else:
                    i += 1
            return blocks

        # Function to detect wall rectangles from a blurred image
        def detectWallRect2(blur, pillarID):
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            result = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.BORDER_DEFAULT)
            rcnt, hierarchy = result if len(result) == 2 else result[1:3]
            walls = []
            for i in rcnt:
                x, y, w, h = cv2.boundingRect(i)
                if ((x > 10 and pillarID == REDBLOCKID) or 
                    (pillarID == GREENBLOCKID and (x + w < frame.shape[2] - 20 or w > 100))) and w > 15 and h > 5:
                    area = cv2.contourArea(i)  
                    if (area > w * h * 0.36):
                        walls.append([x, y, w, h])
            walls.sort(reverse=False, key=lambda w: w[0])
            
            # Merge overlapping walls
            ind, i = len(walls) - 1, 0
            while i < ind:
                if walls[i][0] + walls[i][2] > walls[i + 1][0] - 15:
                    walls[i][2] += walls[i + 1][2] + walls[i + 1][0] - 12
                    walls.pop(i + 1)
                    ind -= 1
                else:
                    i += 1
            walls.sort(reverse=True, key=wallSortingFunc)
            return walls

        # Function to detect wall rectangles from a specific region in the HSV image
        def detectWallRect(hsv, cuts):
            blur = cv2.GaussianBlur(hsv[cuts[2] : cuts[2] + cuts[3], cuts[0]: cuts[0] + cuts[1]].copy(), (13, 13), cv2.BORDER_REFLECT_101)
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            result = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.BORDER_DEFAULT)
            rcnt, hierarchy = result if len(result) == 2 else result[1:3]
            walls = []
            for i in rcnt:
                x, y, w, h = cv2.boundingRect(i)
                if (w > 10 and h > 5):
                    area = cv2.contourArea(i)
                    if (area > w * h * 0.36):
                        walls.append([x, w, y, h])
            walls.sort(reverse=False, key=wallSortingFunc)   
            return walls[0] if walls != [] else []

        # Function to detect lines representing walls
        def detectWallLines(hsv, wlc):
            blur = cv2.GaussianBlur(hsv[wlc[2] : wlc[2] + wlc[3], wlc[0]: wlc[0] + wlc[1]].copy(), (13, 13), cv2.BORDER_REFLECT_101)
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            canny = cv2.Canny(maskb, 50, 140)
            lines = cv2.HoughLinesP(canny, 1, pi/180, 40, minLineLength=20, maxLineGap=45)
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

        # Function to get wall information from HSV image
        def wallInfo(hsv, ccc):
            blur = cv2.GaussianBlur(hsv[ccc[2] : ccc[2] + ccc[3], ccc[0]: ccc[0] + ccc[1]].copy(), (13, 13), cv2.BORDER_REFLECT_101)
            maskb = cv2.inRange(blur, lowerBlack, upperBlack)
            result = cv2.findContours(maskb, cv2.RETR_EXTERNAL, cv2.BORDER_DEFAULT)
            cnt, hierarchy = result if len(result) == 2 else result[1:3]
            coloredArea = 0
            for i in cnt: 
                coloredArea += cv2.contourArea(i) 
                cv2.fillPoly(frame[ccc[2]:FRAME_HEIGHT, ccc[0]:FRAME_WIDTH], pts=[i], color=(255, 255, 255))
            return coloredArea

        # Function to calculate the high and low Y values for a block
        def calculate_yH_yL(block):      
            try:
                yH = block[1] + int(block[3]/1.5)
                if yH < 0:
                    yH = block[1]
                yL = block[1] + block[3] + int(block[3]/2.1)
                if yL - yH < 15:
                    yL += 15 - (yL - yH)
                return (yH, yL)
            except Exception as e:
                print("Calculate yh and yl", e)
                print("Block ", block)
                raise e

        # Function to calculate the overlap ratio between two boxes
        def calculateOverlap(box1, box2):
            if len(box1) == 5:
                x1, y1, w1, h1, _ = box1
            elif len(box1) == 6:
                x1, y1, w1, h1, _, _ = box1
            if len(box2) == 5:
                x2, y2, w2, h2, _ = box2
            elif len(box2) == 6:
                x2, y2, w2, h2, _, _ = box2
            overlap_x = max(0, min(x1 + w1, x2 + w2) - max(x1, x2))
            overlap_y = max(0, min(y1 + h1, y2 + h2) - max(y1, y2))
            overlapArea = overlap_x * overlap_y
            area1 = w1 * h1
            area2 = w2 * h2
            overlapRatio = overlapArea / min(area1, area2)
            return overlapRatio

        count = 0
        imH, imW = 480, 640
        lastMagentas = []
        magenta = []
        colors = {GREENBLOCKID : tuple((0, 220, 105)), REDBLOCKID : tuple((25, 78, 255)), MAGENTAID : tuple((255, 0, 255))}
        estimatedDistanceThreshold = 60
        
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.connect("ipc:///tmp/frame")
        socket.setsockopt_string(zmq.SUBSCRIBE, '')
        points = array([[125, 960], [170, 915], [330, 780], [1020, 760], [1245, 825], [1280, 960]]) // 2
        points2 = array([[0, 0], [0, 190], [0, 335], [160, 280], [320, 245], [650, 190], [750, 190], [950, 225], [1100, 245], [1280, 290], [1280, 0]]) // 2

        
        
        while not(self.INTERRUPT):
            try:
                start = perf_counter()
                #print(f"Waited for {(perf_counter() - start)*1000}ms")
                #start = perf_counter()
                leftWalls, rightWalls = [], []
                
                try:
                    frame_data = socket.recv(zmq.NOBLOCK)  # Non-blocking receive
                except zmq.Again:
                    continue  # Skip to the next iteration if no new frame is available
                buffer = pickle.loads(frame_data)  # Unpickle the data
                frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)  # Decode JPEG image
                frame = cv2.fillPoly(cv2.flip(frame, -1), pts=[points, points2], color=(255, 255, 255))
                #frame = cv2.resize(self.image.copy(), (640, 480), interpolation = cv2.INTER_NEAREST)
                #frame = self.image.copy()
                
                image_resized = cv2.resize(frame, (width, height))
                image_resized = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
                input_data = np.expand_dims(image_resized, axis = 0)
                #input_data = (np.float32(input_data) - input_mean) / input_std
                interpreter.set_tensor(input_details[0]['index'], input_data)
                interpreter.invoke()
                boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] 
                classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0]
                scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0]
                blocks = []
                for i in range(len(scores)):
                    if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
                        ymin = int(max(1,(boxes[i][0] * imH)))
                        xmin = int(max(1,(boxes[i][1] * imW)))
                        ymax = int(min(imH,(boxes[i][2] * imH)))
                        xmax = int(min(imW,(boxes[i][3] * imW)))
                        w = (xmax - xmin)
                        h = (ymax - ymin)
                        object_name = labels[int(classes[i])]
                        if  (h / w > 2.5 and scores[i] < 38) or (object_name == "red" and ( (180 < xmin < 440 and ymin > 250 and ymax > 310 and w < 95) or (
                            xmin > 320 and ymin > 240 and ymax > 310 and w < 100) or (ymin > 220 and ymax < 300 and h/w < 0.75 and w < 30) or (w > 320)) or (
                                xmin < 30 and xmax < 100 and w < 65 and ymin > 300 and ymax > 440) or (260 < xmin < 500 and h / w > 1.75 and h > 150 and ymin < 210 and ymax > 325) ) or (
                                object_name == "green" and ( (ymax > 400 and ymin > 370 and w < 60 and xmin < 65) or (w > 320) or (ymin > 220 and ymax < 300 and h/w < 0.75 and w < 30) or (xmax < 230 and h/w < 1 and w < 50 and ymin > 200 and ymax < 300) )):
                            print(f"{object_name} LINE FILTERED")
                            continue
                        if object_name == "red":
                            if self.lap >= 3 and self.direction == ANTICLOCKWISE and not(time() - self.timeAllSameColorPark < 2.5 and obstacleHeight * focalLength / h < estimatedDistanceThreshold):
                                _id = GREENBLOCKID
                            else:
                                _id = REDBLOCKID
                        elif object_name == "green":
                            if self.lap >= 3 and self.direction == CLOCKWISE and not(time() - self.timeAllSameColorPark < 2.5 and obstacleHeight * focalLength / h < estimatedDistanceThreshold):
                                _id = REDBLOCKID
                            else:
                                _id = GREENBLOCKID
                        elif object_name == "magenta":
                            _id = MAGENTAID
                        blocks.append([xmin, ymin, w, h, _id, int(scores[i] * 100)])
                
                ind, i = len(blocks) - 1, 0
                while i < ind:
                    if calculateOverlap(blocks[0], blocks[1]):
                        x = min(blocks[i][0], blocks[i + 1][0])
                        y = min(blocks[i][1], blocks[i + 1][1])
                        x2 = max(blocks[i][0] + blocks[i][2], blocks[i + 1][0] + blocks[i + 1][2])
                        y2 = max(blocks[i][1] + blocks[i][3], blocks[i + 1][1] + blocks[i + 1][3])
                        blocks[i][0], blocks[i][1], blocks[i][2], blocks[i][3] = x, y, x2 - x, y2 - y
                        blocks.pop(i + 1)
                        #print("Two BLOCKS got merged!")
                        ind -= 1
                    else:
                        i += 1
                
                #print(f"{Style.BRIGHT}{Fore.RED}Processing time: {(perf_counter() - start) * 1000}ms{Style.RESET_ALL}")
                #blur = cv2.GaussianBlur(hsv.copy(), (5, 5), 0, cv2.BORDER_REFLECT_101) #(15, 15)
                #mask1r = cv2.inRange(blur.copy(), lowerRed1, upperRed1)
                #maskr = cv2.inRange(blur.copy(), lowerRed2, upperRed2)
                #maskr = cv2.erode(maskr, kernelErode, iterations = 2)
                #maskr = cv2.dilate(maskr, kernelDilate, iterations = 2)
                #maskr = mask2r#cv2.bitwise_or(mask1r, mask2r)
                #blocks = detectObstacle(maskr, REDBLOCKID)
                #Green pillar
                #maskg = cv2.inRange(blur.copy(), lowerGreen, upperGreen)
                #maskg = cv2.erode(maskg, kernelErode, iterations = 1)
                #blocks.extend(detectObstacle(maskg, GREENBLOCKID))
                
                lastM = magenta
                lenLastM = len(lastM)
                magenta = []
                toPop = []
                for i, block in enumerate(blocks):
                    if  block[4] == MAGENTAID or (
                        (block[2] / block[3] > 1.6  and block[4] == REDBLOCKID) or (self.direction == ANTICLOCKWISE and lenLastM > 0 and block[0]+block[2] > 550) or (self.direction == CLOCKWISE and lenLastM > 0 and block[0]+block[2]<120)):
                        # and not(block[2] > 350)
                        toPop.append(i)
                        cv2.rectangle(frame, (block[0], block[1]), (block[0] + block[2], block[1] + block[3]), (0, 0, 0), -1)
                        cv2.putText(frame, f"Magenta: {block[5]}", (block[0], block[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[block[4]], 2, cv2.LINE_AA)
                for i in toPop[::-1]:
                    magenta.append(blocks.pop(i))
                
                hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
                
                
                for block in blocks:
                    if block[4] == GREENBLOCKID:
                        text = f"Green: {block[5]}"
                    elif block[4] == REDBLOCKID:
                        text = f"Red: {block[5]}"
                    else:
                        text = f"Magenta: {block[5]}"
                    cv2.rectangle(frame, (block[0], block[1]), (block[0] + block[2], block[1] + block[3]), colors[block[4]], 1)
                    cv2.putText(frame, text, (block[0], block[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[block[4]], 1, cv2.LINE_AA)
                blocks.sort(reverse = True, key = closerBlockFilter) 
                magenta.sort(reverse = True, key = closerBlockFilter)
                #print(blocks if blocks != [] else "0 Obstacles found")
                # Checking whether or not obstacles were found and give the position of the steering
                if len(blocks) > 0:
                    yH, yL = calculate_yH_yL(blocks[0])
                    if blocks[0][4] == REDBLOCKID:
                        #print("Red block closer")
                        image = frame[yH:yL, blocks[0][0] + blocks[0][2] if blocks[0][0] + blocks[0][2] < 640 else 639:640]
                        
                        #if self.direction != CLOCKWISE and len(magenta) > 0 and magenta[0][0]+blocks[0][0] > 320:
                        #    magenta.sort(key = lambda x: x[0])
                        #    rightWalls = [[magenta[0][0], magenta[0][1] - blocks[0][1], magenta[0][2], magenta[0][3]]]# - blocks[0][0]
                        #    print("HERE")
                        #else:
                        rightWalls = detectWallRect2(image, REDBLOCKID)
                        
                        for wall in rightWalls:
                            image = cv2.rectangle(image, (wall[0], wall[1]), (wall[0] + wall[2], wall[1] + wall[3] - 1), (255, 255, 0), 1)
                        image = frame[yH+5:yL+5, 1:blocks[0][0] if blocks[0][0] > 1 else 2]
                        leftWalls = detectWallRect2(image, GREENBLOCKID)
                        for wall in leftWalls:
                            image = cv2.rectangle(image, (wall[0], wall[1]), (wall[0] + wall[2], wall[1] + wall[3] - 1), (255, 255, 0), 1)
                        
                    if blocks[0][4] == GREENBLOCKID:
                        #print("Green block closer")
                        image = frame[yH-5:yL+10, blocks[0][0] + blocks[0][2] if blocks[0][0] + blocks[0][2] < 640 else 639:640]
                        rightWalls = detectWallRect2(image, REDBLOCKID)
                        for wall in rightWalls:
                            image = cv2.rectangle(image, (wall[0], wall[1]), (wall[0] + wall[2], wall[1] + wall[3] - 1), (255, 255, 0), 1)
                        image = frame[yH:yL, 1:blocks[0][0] if blocks[0][0] > 1 else 2]
                    
                        #if self.direction != ANTICLOCKWISE and len(magenta) > 0 and magenta[0][0] < 320:
                        #    magenta.sort(key = lambda x: x[0], reverse = True)
                        #    leftWalls = [[magenta[0][0], magenta[0][1] - blocks[0][1], magenta[0][2], magenta[0][3]]]
                        #else:
                        leftWalls = detectWallRect2(image, GREENBLOCKID)
                        
                        for wall in leftWalls:
                            image = cv2.rectangle(image, (wall[0], wall[1]), (wall[0] + wall[2], wall[1] + wall[3] - 1), (255, 255, 0), 1)
                
                    if blocks[0][4] == REDBLOCKID:
                        if rightWalls != []:
                            if self.direction == ANTICLOCKWISE:
                                virtualPoint = int( blocks[0][0] + blocks[0][2] + (rightWalls[0][0] / 2) ) 
                            else:
                                virtualPoint = int( blocks[0][0] + (rightWalls[0][0] / 2) )
                        else:
                            virtualPoint = int(blocks[0][0] + blocks[0][2] * 3.5)# *3.5
                        frame = cv2.rectangle(frame, (virtualPoint - 4, blocks[0][1] + blocks[0][3] - 4), (virtualPoint + 4, blocks[0][1] + blocks[0][3] + 4), (0, 255, 255), 2)

                    elif blocks[0][4] == GREENBLOCKID:
                        if leftWalls != []:
                            if self.direction == CLOCKWISE:
                                virtualPoint = int( (leftWalls[0][0] + leftWalls[0][2] + blocks[0][0]) / 2 )
                            else:
                                virtualPoint = int( (leftWalls[0][0] + leftWalls[0][2] + blocks[0][0]) / 2 )
                        else:
                            virtualPoint = int(blocks[0][0] - blocks[0][2] * 2.5) # *3
                        frame = cv2.rectangle(frame, (virtualPoint - 4, blocks[0][1] + blocks[0][3] - 4), (virtualPoint + 4, blocks[0][1] + blocks[0][3] + 4), (0, 255, 255), 2)
                    # Getting wall lines
                    if blocks[0][4] == REDBLOCKID:  
                        frame = cv2.rectangle(frame, (wlcR[0], wlcR[2]), (wlcR[0] + wlcR[1], wlcR[2] + wlcR[3]), (150, 30, 190), 1)
                        line = detectWallLines(hsv.copy(), wlcR)
                        if len(line) > 0:
                            line = (line[0] + wlcR[0], line[1] + wlcR[2], line[2] + wlcR[0], line[3] + wlcR[2])
                            frame = cv2.line(frame, (line[0], line[1]), (line[2], line[3]), (180, 10, 180), 3)
                        frame = cv2.rectangle(frame, (cccR[0], cccR[2]), (cccR[0] + cccR[1], cccR[2] + cccR[3]), (30, 210, 80), 3)
                        frame = cv2.rectangle(frame, (lwR[0], lwR[2]), (lwR[0] + lwR[1], lwR[2] + lwR[3]), (15, 100, 215), 3)
                        frame = cv2.rectangle(frame, (lwR2[0], lwR2[2]), (lwR2[0] + lwR2[1], lwR2[2] + lwR2[3]), (0, 50, 225), 3)
                        coloredAreaMid = int(wallInfo(hsv, cccR))
                        coloredAreaLower = int(wallInfo(hsv, lwR))
                        coloredAreaLower2 = int(wallInfo(hsv, lwR2))
                    elif blocks[0][4] == GREENBLOCKID:
                        frame = cv2.rectangle(frame, (wlcL[0], wlcL[2]), (wlcL[0] + wlcL[1], wlcL[2] + wlcL[3]), (150, 30, 190), 1)  
                        line = detectWallLines(hsv.copy(), wlcL)
                        if len(line) > 0:
                            line = (line[0], line[1] + wlcL[2], line[2], line[3] + wlcL[2])
                            frame = cv2.line(frame, (line[0], line[1]), (line[2], line[3]), (180, 10, 180), 3)
                        frame = cv2.rectangle(frame, (cccL[0], cccL[2]), (cccL[0] + cccL[1], cccL[2] + cccL[3]), (30, 210, 80), 3)
                        frame = cv2.rectangle(frame, (lwL[0], lwL[2]), (lwL[0] + lwL[1], lwL[2] + lwL[3]), (15, 100, 215), 3)
                        frame = cv2.rectangle(frame, (lwL2[0], lwL2[2]), (lwL2[0] + lwL2[1], lwL2[2] + lwL2[3]), (0, 50, 225), 3)
                        coloredAreaMid = int(wallInfo(hsv, cccL))
                        coloredAreaLower = int(wallInfo(hsv, lwL))
                        coloredAreaLower2 = int(wallInfo(hsv, lwL2))
                if rightWalls == []:
                    rightWalls.append((0, 0, 0, 0))
                if leftWalls == []:
                    leftWalls.append((0, 0, 0, 0))
                else:
                    if self.direction == CLOCKWISE and block[4] == GREENBLOCKID and (leftWalls[0][0] + leftWalls[0][2] - blocks[0][0]) + 15 > 0:
                        if len(leftWalls) == 1:
                            leftWalls = [(0, 0, 0, 0)]
                        else:
                            leftWalls.pop(0)
                
                #handleParking(magenta, frame)
                
                if len(blocks) > 1:
                    #avoidObstacle(blocks[0], walls[0], virtualPoint, line, blocks[1], coloredAreaMid, coloredAreaLower, coloredAreaLower2)    
                    self.lock.acquire()
                    self.shared_values[0], self.shared_values[1], self.shared_values[2], self.shared_values[3], self.shared_values[4] = blocks[0][0], blocks[0][1], blocks[0][2], blocks[0][3], blocks[0][4]
                    self.shared_values[5], self.shared_values[6], self.shared_values[7], self.shared_values[8] = rightWalls[0]
                    self.shared_values[9], self.shared_values[10], self.shared_values[11], self.shared_values[12], self.shared_values[13] = virtualPoint, line[0], line[1], line[2], line[3]
                    self.shared_values[14], self.shared_values[15], self.shared_values[16], self.shared_values[17], self.shared_values[18] = blocks[1][0], blocks[1][1], blocks[1][2], blocks[1][3], blocks[1][4]
                    self.shared_values[19], self.shared_values[20], self.shared_values[21] = coloredAreaMid, coloredAreaLower, coloredAreaLower2
                    self.shared_values[22], self.shared_values[23], self.shared_values[24], self.shared_values[25] = leftWalls[0]
                    if len(magenta) > 0:
                        self.shared_values[30] = magenta
                    else:
                        self.shared_values[30] = []
                    self.shared_values[31] =  2
                    self.lock.release()
                elif len(blocks) > 0:
                    #avoidObstacle(blocks[0], walls[0], virtualPoint, line, [-1, -1, -1, -1, -1], coloredAreaMid, coloredAreaLower, coloredAreaLower2)    
                    self.lock.acquire()
                    self.shared_values[0], self.shared_values[1], self.shared_values[2], self.shared_values[3], self.shared_values[4] = blocks[0][0], blocks[0][1], blocks[0][2], blocks[0][3], blocks[0][4]
                    self.shared_values[5], self.shared_values[6], self.shared_values[7], self.shared_values[8] = rightWalls[0]
                    self.shared_values[9], self.shared_values[10], self.shared_values[11], self.shared_values[12], self.shared_values[13] = virtualPoint, line[0], line[1], line[2], line[3]
                    self.shared_values[14] = self.shared_values[15] = self.shared_values[16] = self.shared_values[17] = self.shared_values[18] = -1
                    self.shared_values[19], self.shared_values[20], self.shared_values[21] = coloredAreaMid, coloredAreaLower, coloredAreaLower2
                    self.shared_values[22], self.shared_values[23], self.shared_values[24], self.shared_values[25] = leftWalls[0]
                    if len(magenta) > 0:
                        self.shared_values[30] = magenta
                    else:
                        self.shared_values[30] = []
                    self.shared_values[31] =  2
                    self.lock.release()
                else:
                    if self.drawDebug:
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
                    #print(f"Colored area:\nleftMid: {coloredAreaLeftMid}, rightMid: {coloredAreaRightMid},\nleftIn: {coloredAreaLeftIn}, rightIn: {coloredAreaRightIn},\nleftOut: {coloredAreaLeftOut}, rightOut: {coloredAreaRightOut},\nfront: {coloredAreaFront},front2: {coloredAreaFront2}")
                    #print(f"lower left: {coloredAreaLowerLeft}, lower right: {coloredAreaLowerRight}")
                    # Getting wall rects
                    leftRect = detectWallRect(hsv, mpcL)
                    rightRect = detectWallRect(hsv, mpcR)
                    if len(leftRect) > 0:
                        leftRect = (leftRect[0], leftRect[1], leftRect[2] + mpcL[2], leftRect[3])
                        frame = cv2.rectangle(frame, (leftRect[0], leftRect[2]), (leftRect[0] + leftRect[1], leftRect[2] + leftRect[3]), (150, 120, 25), 3)
                    if len(rightRect) > 0:
                        rightRect = (rightRect[0] + mpcL[1], rightRect[1], rightRect[2] + mpcR[2], rightRect[3])
                        frame = cv2.rectangle(frame, (rightRect[0], rightRect[2]), (rightRect[0] + rightRect[1], rightRect[2] + rightRect[3]), (150, 120, 25), 3)
                    #print(f"Left rect: {leftRect}, right rect: {rightRect}")
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
                    #print(f"Left line: {leftLine}, right line: {rightLine}")                
                    if leftRect == []:
                        leftRect = [0, 0, 0, 0]
                    if rightRect == []:
                        rightRect = [0, 0, 0, 0]
                    #makeDecision(coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn, 
                    #coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn, coloredAreaFront,
                    #leftRect, rightRect, leftLine, rightLine,
                    #coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2)
                    self.lock.acquire()
                    self.shared_values[0], self.shared_values[1], self.shared_values[2], self.shared_values[3], self.shared_values[4], self.shared_values[5] = coloredAreaLeftOut, coloredAreaLeftMid, coloredAreaLeftIn, coloredAreaRightOut, coloredAreaRightMid, coloredAreaRightIn
                    self.shared_values[6] = coloredAreaFront
                    self.shared_values[7], self.shared_values[8], self.shared_values[9], self.shared_values[10] = leftRect
                    self.shared_values[11], self.shared_values[12], self.shared_values[13], self.shared_values[14] = rightRect
                    self.shared_values[15], self.shared_values[16], self.shared_values[17], self.shared_values[18] = leftLine
                    self.shared_values[19], self.shared_values[20], self.shared_values[21], self.shared_values[22] = rightLine
                    self.shared_values[23], self.shared_values[24], self.shared_values[25] = coloredAreaLowerLeft, coloredAreaLowerRight, coloredAreaFront2
                    self.shared_values[26], self.shared_values[27], self.shared_values[28], self.shared_values[29] = centralLine
                    if len(magenta) > 0:
                        self.shared_values[30] = magenta
                    else:
                        self.shared_values[30] = []
                    self.shared_values[31] =  1
                    self.lock.release()
                #End of the logic
                self.frame = frame
                self.PROCESSED = True
                self.TOBESAVED = True
                print(Style.RESET_ALL, sep = "", end = "")
                processingTime = perf_counter() - start
                self.processingTimeList.append(processingTime)
                #print(f"{Style.BRIGHT}{Fore.BLUE}Processing time: {processingTime * 1000}ms{Style.RESET_ALL}")
            #except UnboundLocalError:
            #    print("Unbound local error finalImageProcessing")
            except KeyboardInterrupt:
                pass
    def saveFrame(self):
        from datetime import datetime
        imagesCounter = 0
        
        # Continuously save frames while not interrupted
        while not(self.INTERRUPT):
            if self.TOBESAVED:
                # Create a unique filename based on the current datetime and a counter
                uniqueTimeName = "__" + str(datetime.now()).replace(" ", ";").replace(".", ";").replace(":", "-") + "__" + str(imagesCounter)
                
                # Save the current frame to the 'images' directory with high quality
                cv2.imwrite("images/" + uniqueTimeName + ".jpg", self.frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                
                # Save the raw image to the 'rawImages' directory with high quality
                cv2.imwrite("rawImages/" + uniqueTimeName + ".jpg", self.image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                
                # Increment the image counter
                imagesCounter += 1
                
                # Reset the flag to indicate that the frame has been saved
                self.TOBESAVED = False
            
            # Sleep briefly to avoid high CPU usage
            sleep(0.1)

    def run(self):
        # Create and start threads for frame capturing, processing, and optionally displaying and saving frames
        getFrameThread = Thread(target=self.getFrame)        
        processFrameThread = Thread(target=self.processFrame)
        
        getFrameThread.start()  # Start the frame capture thread
        processFrameThread.start()  # Start the frame processing thread
        
        if self.showFrameFlag:
            # If the flag is set, start a thread to display frames
            showFrameThread = Thread(target=self.showFrame)
            showFrameThread.start()
        
        if self.saveFrameFlag:
            # If the flag is set, start a thread to save frames
            saveFrameThread = Thread(target=self.saveFrame)
            saveFrameThread.start()

    def stop(self):
        try:
            # Set the interrupt flag to stop the frame processing loop
            self.INTERRUPT = True
            self.camera_feed_process.terminate()
            # Calculate and log the average processing time
            average = sum(self.processingTimeList) / len(self.processingTimeList)
            with open("processingTimeAverage.txt", "a") as f:
                f.write(f"\n{average}")
            print(f"Average processing time: {average}")
        except Exception as e:
            print(f"Image processing stop error: {e}")
        
        try:
            # Close any OpenCV windows
            cv2.destroyAllWindows()
        except:
            # If there is an error closing windows, just pass
            pass

if __name__ == '__main__':
    lock = Lock()
    shared_values = [i for i in range(40)]
    
    # Initialize the ImageProcessing class with shared values and a lock
    imageProcessing = ImageProcessing(shared_values, lock, True, False)
    
    # Start the image processing threads
    imageProcessing.run()
    
    sleep(1)  # Allow some time for initialization
    
    while True:
        try:
            # Main loop to keep the program running
            # Uncomment the following lines if needed for debugging
            # print(f"Shared values: {list(shared_values)}")
            # print(imageProcessing.frame)
            sleep(1)
        except KeyboardInterrupt:
            # Exit the loop if a keyboard interrupt (Ctrl+C) is detected
            break
    
    # Stop the image processing
    imageProcessing.stop()
    exit()
