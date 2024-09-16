from numpy import array, uint8, pi

#Environment based constants
lowLuminosity, midLuminosity, highLuminosity = 35, 45, 55

# Constants
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
lowerBlack = array([0, 0, 0], uint8)
upperBlack = array([255, 255, 65], uint8) # 55 o 35, dipende dalla luminosità
# Set 1 of wall sensors 
cccL = (190//2, 120//2, 340//2, 60//2)
cccR = (FRAME_WIDTH - cccL[0] - cccL[1] + 40//2, cccL[1] + 30//2, cccL[2] - 40//2, cccL[3] + 20//2)
correctionAreaPercentageMid = cccL[1] * cccL[3] * 0.2
# Set 2 of wall sensors
cccL2 = (cccL[0] + cccL[1], (130)//2, cccL[2] - 10//2, cccL[3] - 20//2)
cccR2 = (FRAME_WIDTH - cccL2[0] - cccL2[1] + int(40//2), cccL2[1] + 30//2, cccL2[2] - 40//2, cccL2[3] + 40//2)
correctionAreaPercentageIn = cccL2[1] * cccL2[3] * 0.2
# Set 3 of wall sensors
cccL3 = ((cccL[0] - int(cccL[1] // 1.5)), int(cccL[1] // 1.5), cccL[2] + 10//2, cccL[3] + 20//2)
cccR3 = (FRAME_WIDTH - cccL3[0] - cccL3[1] + 40//2, cccL3[1] + 30//2, cccL3[2] - 40//2, cccL3[3] + 20//2)
correctionAreaPercentageOut = cccL3[1] * cccL3[3] * 0.22
# Middle points coordinates
mpcL = (0, FRAME_WIDTH // 2, 360//2 + 30, 20//2)
mpcR = (FRAME_WIDTH - mpcL[0] - mpcL[1], mpcL[1], mpcL[2], mpcL[3])
# Wall lines coordinates
wlcL = (0, 320//2, 380//2, 380//2)
wlcR = (FRAME_WIDTH - wlcL[0] - wlcL[1] + 20//2, wlcL[1] + 30//2, wlcL[2] - 40//2, wlcL[3] + 20//2)
# cccf 
wcccf = 120//2
hcccf = 60//2
cccf = (FRAME_WIDTH // 2 - wcccf // 2, wcccf, cccL[2] - int(hcccf // 3), hcccf)
correctionAreaPercentageFront = cccf[1] * cccf[3] * 0.58
# cccf2
wcccf2 = 120//2
hcccf2 = 60//2
cccf2 = (FRAME_WIDTH // 2 - wcccf2 // 2, wcccf2, cccL[2] - int(hcccf2 * 2.1), hcccf2)
correctionAreaPercentageFront2 = cccf2[1] * cccf2[3] * 0.55

# Front flag 2:
# Un po' più alto del primo, serve per capire se può correggere la sua posizione
# "go right" o "go left" specifica di due casi (False corregge, True non corregge) 
lw = 60//2
lh = 100//2
lwL = (0, lw, 420//2, lh)
lwR = (FRAME_WIDTH - lwL[1], lwL[1], lwL[2], lwL[3])
correctionAreaPercentageLower = lwL[1] * lwL[3] * 0.34

#####
wlcC = (cccf[0], cccf[1], cccf[2] - 15, cccf[3] + 40)

lw2 = 30
lh2 = 50
lwL2 = (0, lw, 270, lh)
lwR2 = (FRAME_WIDTH - lwL2[1], lwL2[1], lwL2[2], lwL2[3])
correctionAreaPercentageLower2 = lwL2[1] * lwL2[3] * 0.34

cccL = (cccL[0] + 35, cccL[1], cccL[2] - 10, cccL[3]) #45
cccL2 = (cccL2[0] + 35, cccL2[1], cccL2[2] - 10, cccL2[3])#45
cccL3 = (cccL3[0] + 35, cccL3[1], cccL3[2] - 10, cccL3[3])#45
lwL = (lwL[0] + 35, lwL[1], lwL[2], lwL[3])#45
cccR = (cccR[0] - 20, cccR[1] - 10, cccR[2], cccR[3] - 10) 
cccR2 = (cccR2[0] - 20, cccR2[1] - 10, cccR2[2], cccR2[3] - 10)
cccR3 = (cccR3[0] - 20, cccR3[1] - 10, cccR3[2], cccR3[3] - 10)
lwR = (lwR[0] - 20, lwR[1], lwR[2] - 10, lwR[3] - 10)
# Variabili
CLOCKWISE = 1
ANTICLOCKWISE = -1
