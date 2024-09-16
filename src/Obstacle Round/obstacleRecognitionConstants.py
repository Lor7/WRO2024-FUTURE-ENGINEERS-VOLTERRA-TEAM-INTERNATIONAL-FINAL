from wallRecognitionConstants import *
from numpy import ones

# Define color thresholds for detecting red and green obstacles.
lowerRed1 = array([0, 80, 85], uint8)
upperRed1 = array([9, 235, 231], uint8)
lowerRed2 = array([168, 58, 42], uint8)
upperRed2 = array([180, 225, 221], uint8)
lowerGreen = array([32, 35, 30], uint8)
upperGreen = array([100, 238, 235], uint8)

# Define IDs for different types of obstacles.
GREENBLOCKID = 1
REDBLOCKID = 2
MAGENTAID = 3
WALLID = 5
WALL_WAS_SEEN = 1
PILLAR_WAS_SEEN = 2
DEFAULT_SHARED_MEMORY_FLAG = 0
ANTICLOCKWISE = -1

focalLength = 633.3333333333334 / 2
realMeasure = 10
obstacleWidth = 5
obstacleHeight = 10

# Define directional constants.
CLOCKWISE = 1
ANTICLOCKWISE = -1

# Define kernel sizes for morphological operations in image processing.
kernelErode = ones((5, 5), uint8)
kernelDilate = ones((3, 3), uint8)

HALF_FRAME_WIDTH = FRAME_WIDTH // 2

# Lambda function to filter blocks based on their position and size.
closerBlockFilter = lambda l: l[1] + l[3] - abs(((305 - l[0] - l[2] / 2) / HALF_FRAME_WIDTH) ** 2 * 50)
