from numpy import array, uint8, pi

# Environment-based constants for luminosity thresholds
lowLuminosity, midLuminosity, highLuminosity = 35, 45, 55

# Image Frame Constants
FRAME_WIDTH, FRAME_HEIGHT = 640, 480

# Color Range Constants for Black Detection
lowerBlack = array([0, 0, 0], uint8)  # Lower bound for black color
upperBlack = array([255, 255, 50], uint8)  # Upper bound for black color (varies with luminosity)

# Wall Sensor Areas - Set 1
# Coordinates for the first set of wall sensors
cccL = (190 // 2, 120 // 2, 340 // 2, 60 // 2)  # Left sensor coordinates
cccR = (FRAME_WIDTH - cccL[0] - cccL[1] + 40 // 2, cccL[1] + 30 // 2, cccL[2] - 40 // 2, cccL[3] + 20 // 2)  # Right sensor coordinates
correctionAreaPercentageMid = cccL[1] * cccL[3] * 0.2  # Correction area percentage for mid-sensors

# Wall Sensor Areas - Set 2
# Coordinates for the second set of wall sensors
cccL2 = (cccL[0] + cccL[1], (130) // 2, cccL[2] - 10 // 2, cccL[3] - 20 // 2)  # Left sensor coordinates
cccR2 = (FRAME_WIDTH - cccL2[0] - cccL2[1] + int(40 // 2), cccL2[1] + 30 // 2, cccL2[2] - 40 // 2, cccL2[3] + 40 // 2)  # Right sensor coordinates
correctionAreaPercentageIn = cccL2[1] * cccL2[3] * 0.2  # Correction area percentage for inward sensors

# Wall Sensor Areas - Set 3
# Coordinates for the third set of wall sensors
cccL3 = ((cccL[0] - int(cccL[1] // 1.5)), int(cccL[1] // 1.5), cccL[2] + 10 // 2, cccL[3] + 20 // 2)  # Left sensor coordinates
cccR3 = (FRAME_WIDTH - cccL3[0] - cccL3[1] + 40 // 2, cccL3[1] + 30 // 2, cccL3[2] - 40 // 2, cccL3[3] + 20 // 2)  # Right sensor coordinates
correctionAreaPercentageOut = cccL3[1] * cccL3[3] * 0.22  # Correction area percentage for outward sensors

# Middle Points Coordinates
# Coordinates for the middle points used for alignment
mpcL = (0, FRAME_WIDTH // 2, 360 // 2 + 30, 20 // 2)  # Left middle point
mpcR = (FRAME_WIDTH - mpcL[0] - mpcL[1], mpcL[1], mpcL[2], mpcL[3])  # Right middle point

# Wall Lines Coordinates
# Coordinates for wall lines used for detection and navigation
wlcL = (0, 320 // 2, 380 // 2, 380 // 2)  # Left wall line
wlcR = (FRAME_WIDTH - wlcL[0] - wlcL[1] + 20 // 2, wlcL[1] + 30 // 2, wlcL[2] - 40 // 2, wlcL[3] + 20 // 2)  # Right wall line

# Front Correction Area - First Set
# Coordinates and correction area for the front of the robot
wcccf = 120 // 2
hcccf = 60 // 2
cccf = (FRAME_WIDTH // 2 - wcccf // 2, wcccf, cccL[2] - int(hcccf // 3), hcccf)  # Front correction area
correctionAreaPercentageFront = cccf[1] * cccf[3] * 0.58  # Correction area percentage for the front

# Front Correction Area - Second Set
# Coordinates and correction area for an alternative front position
wcccf2 = 120 // 2
hcccf2 = 60 // 2
cccf2 = (FRAME_WIDTH // 2 - wcccf2 // 2, wcccf2, cccL[2] - int(hcccf2 * 2.1), hcccf2)  # Front correction area 2
correctionAreaPercentageFront2 = cccf2[1] * cccf2[3] * 0.55  # Correction area percentage for the front 2

# Lower Flag Areas
# Areas for detecting lower flags that help in positioning
lw = 60 // 2
lh = 100 // 2
lwL = (0, lw, 420 // 2, lh)  # Left lower flag area
lwR = (FRAME_WIDTH - lwL[1], lwL[1], lwL[2], lwL[3])  # Right lower flag area
correctionAreaPercentageLower = lwL[1] * lwL[3] * 0.34  # Correction area percentage for lower flags

# Additional Correction Areas
# Additional areas for further corrections or detection
wlcC = (cccf[0], cccf[1], cccf[2] - 15, cccf[3] + 40)  # Central correction area

lw2 = 30
lh2 = 50
lwL2 = (0, lw, 270, lh)  # Left additional lower flag area
lwR2 = (FRAME_WIDTH - lwL2[1], lwL2[1], lwL2[2], lwL2[3])  # Right additional lower flag area
correctionAreaPercentageLower2 = lwL2[1] * lwL2[3] * 0.34  # Correction area percentage for additional lower flags

# Adjusted Sensor Coordinates
# Adjust coordinates for sensor areas
cccL = (cccL[0] + 35, cccL[1], cccL[2] - 10, cccL[3])  # Adjusted left sensor coordinates
cccL2 = (cccL2[0] + 35, cccL2[1], cccL2[2] - 10, cccL2[3])  # Adjusted second set left sensor coordinates
cccL3 = (cccL3[0] + 35, cccL3[1], cccL3[2] - 10, cccL3[3])  # Adjusted third set left sensor coordinates
lwL = (lwL[0] + 35, lwL[1], lwL[2], lwL[3])  # Adjusted lower flag coordinates
cccR = (cccR[0] - 20, cccR[1] - 10, cccR[2], cccR[3] - 10)  # Adjusted right sensor coordinates
cccR2 = (cccR2[0] - 20, cccR2[1] - 10, cccR2[2], cccR2[3] - 10)  # Adjusted second set right sensor coordinates
cccR3 = (cccR3[0] - 20, cccR3[1] - 10, cccR3[2], cccR3[3] - 10)  # Adjusted third set right sensor coordinates
lwR = (lwR[0] - 20, lwR[1], lwR[2] - 10, lwR[3] - 10)  # Adjusted lower flag coordinates

# Direction Constants
CLOCKWISE = 1  # Constant for clockwise direction
ANTICLOCKWISE = -1  # Constant for anticlockwise direction
