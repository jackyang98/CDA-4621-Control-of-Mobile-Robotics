import time
import cv2 as cv
from ThreadedWebcam import ThreadedWebcam
from UnthreadedWebcam import UnthreadedWebcam
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO
import array
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import random

class Cell:
    def __init__(self, west, north, east, south, visited = False):
        self.west = west
        self.north = north
        self.east = east
        self.south = south
        self.visited = visited
def detectMazeInconsistencies(maze):
    for i in range(3):
        for j in range(4):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 4
            hWall1 = maze[pos1].south
            hWall2 = maze[pos2].north
            assert hWall1 == hWall2, " Cell " + str(pos1) + "'s south wall doesn't equal cell " + str(pos2) + "'s north wall! ('" + str(hWall1) + "' != '" + str(hWall2) + "')"
    for i in range(4):
        for j in range(3):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 1
            vWall1 = maze[pos1].east
            vWall2 = maze[pos2].west
            assert vWall1 == vWall2, " Cell " + str(pos1) + "'s east wall doesn't equal cell " + str(pos2) + "'s west wall! ('" + str(vWall1) + "' != '" + str(vWall2) + "')"
def printMaze(maze, hRes = 4, vRes = 2):
    assert hRes > 0, "Invalid horizontal resolution"
    assert vRes > 0, "Invalid vertical resolution"# Get the dimensions of the maze drawing
    hChars = 4 * (hRes + 1) + 2
    vChars = 4 * (vRes + 1) + 1# Store drawing into a list
    output = [" "] * (hChars * vChars - 1)# Draw top border
    for i in range(1, hChars - 2):
        output[i] = "_"
    for i in range(hChars * (vChars - 1) + 1, hChars * (vChars - 1) + hChars - 2):
        output[i] = " ̄"# Draw left border
    for i in range(hChars, hChars * (vChars - 1), hChars):
        output[i] = "|"# Draw right border
    for i in range(2 * hChars - 2, hChars * (vChars - 1), hChars):
        output[i] = "|"# Draw newline characters
    for i in range(hChars - 1, hChars * vChars - 1, hChars):
        output[i] = "\n"# Draw dots inside maze
    for i in range((vRes + 1) * hChars, hChars * (vChars - 1), (vRes + 1) * hChars):
        for j in range(hRes + 1, hChars - 2, hRes + 1):
            output[i + j] = "·"
    for i in range(4):
        for j in range(4):
            cellNum = i * 4 + j
            if maze[cellNum].visited:
                continue
            origin = (i * hChars * (vRes + 1) + hChars + 1) + (j * (hRes + 1))
            for k in range(vRes):
                for l in range(hRes):
                    output[origin + k * hChars + l] = "?"
    for i in range(3):
        for j in range(4):
            cellNum = i * 4 + j
            origin = ((i + 1) * hChars * (vRes + 1) + 1) + (j * (hRes + 1))
            hWall = maze[cellNum].south
            for k in range(hRes):
                output[origin + k] = "-" if hWall == 'W' else " " if hWall == 'O' else "?"# Draw vertical walls
    for i in range(4):
        for j in range(3):
            cellNum = i * 4 + j
            origin = hChars + (hRes + 1) * (j + 1) + i * hChars * (vRes + 1)
            vWall = maze[cellNum].east
            for k in range(vRes):
                output[origin + k * hChars] = "|" if vWall == 'W' else " " if vWall == 'O' else "?"# Print drawing
    print(''.join(output))

maze = [Cell('?','?','?','?', False),Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False),
        Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False),
        Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False),
        Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False)]
LSERVO = 0
RSERVO = 1
#Variables
ACTUALTIME = 0
LASTTIME = 0
LEFTCOUNT = 0
RIGHTCOUNT = 0
RSV =[5.85464,5.60965,5.600,5.59982,5.4526,5.345599,5.21568,5.091117,4.83616,4.57394,4.33566,3.82513,3.30913,2.79993,2.29062,2.03636,1.27265,1.01818,0.50909,0,0,0,0,-0.50909,-1.01818,-1.27265,-2.03636,-2.29062,-2.79993,-3.30912,-3.82513,-4.33566,-4.57394,-4.83616,-5.09111,-5.34559,-5.21852,-5.46853,-5.55698,-5.59981,-5.60027,-5.85463,-5.87565]
LSV =[5.85464,5.60965,5.600,5.59982,5.4526,5.345599,5.21568,5.091117,4.83616,4.57394,4.33566,3.82513,3.30913,2.79993,2.29062,2.03636,1.27265,1.01818,0.50909,0,0,0,0,-0.50909,-1.01818,-1.27265,-2.03636,-2.29062,-2.79993,-3.30912,-3.82513,-4.33566,-4.57394,-4.83616,-5.09111,-5.34559,-5.21852,-5.46853,-5.55698,-5.59981,-5.60027,-5.85463,-5.87565]

#LSV =[-5.85464,-5.60965,-5.600,-5.59982,-5.4526,-5.345599,-5.21568,-5.091117,-4.83616,-4.57394,-4.33566,-3.82513,-3.30913,-2.79993,-2.29062,-2.03636,-1.27265,-1.01818,-0.50909,0,0,0,0,0.50909,1.01818,1.27265,2.03636,2.29062,2.79993,3.30912,3.82513,4.33566,4.57394,4.83616,5.09111,5.34559,5.21852,5.46853,5.55698,5.59981,5.60027,5.85463,5.87565]
#RSV =[5.85464,5.60965,5.600,5.59982,5.4526,5.345599,5.21568,5.091117,4.83616,4.57394,4.33566,3.82513,3.30913,2.79993,2.29062,2.03636,1.27265,1.01818,0.50909,0,0,0,0,-0.50909,-1.01818,-1.27265,-2.03636,-2.29062,-2.79993,-3.30912,-3.82513,-4.33566,-4.57394,-4.83616,-5.09111,-5.34559,-5.21852,-5.46853,-5.55698,-5.59981,-5.60027,-5.85463,-5.87565]
def calibrateSpeeds():
    x = 1.40
    i = 0
    while (x < 1.61):
        resetCounts()
        pwm.set_pwm(LSERVO, 0, math.floor(x / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(x / 20 * 4096));
        time.sleep(1)

        i = i + 1
        x = x + 0.005
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
def initEncoders():
    # Pins that the encoders are connected to
    global LENCODER
    LENCODER = 17
    global RENCODER
    RENCODER = 18
    
initEncoders()

def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    
    exit()
    
def resetCounts():
    global LEFTCOUNT,RIGHTCOUNT,LASTTIME
    LEFTCOUNT = 0
    RIGHTCOUNT = 0
    LASTTIME = time.monotonic()
    
#Get count of encoders
def getCounts():
    return LEFTCOUNT,RIGHTCOUNT
    
# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    global LEFTCOUNT
    LEFTCOUNT = LEFTCOUNT + 1

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global RIGHTCOUNT
    RIGHTCOUNT = RIGHTCOUNT + 1

# Set encoder pins as input
# Also enable pull-up resistors on the encoder pins
# This ensures a clean 0V and 3.3V is always outputted from the encoders.
GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library, 
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b

# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

# Shutdown all sensors
GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)

time.sleep(0.01)

# Initialize all sensors
lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)

# Connect the left sensor and start measurement
GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the right sensor and start measurement
GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Attach a rising edge interrupt to the encoder pins
GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

FPS_SMOOTHING = 0.9

minH = 166; minS =  96; minV =  121
maxH = 180; maxS = 167; maxV = 255

fps = 0; prev = 0

# Uncomment below to use the slower webcam
capture = ThreadedWebcam() # UnthreadedWebcam()
capture.start()

params = cv.SimpleBlobDetector_Params()
detector = cv.SimpleBlobDetector_create(params)
fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ)
if not fs.isOpened():
		print("No params")
		exit(1)
detector.read(fs.root())
fs.release()

cols = 4
rows = 4
matrix = [["?"]*cols]*rows
        
def turnleft(at):
    v = 0
    Rvalue = 0
    i = 0
    
    if at < LSV[6]:
        v = 1.57
        return v
    
    vel = abs(at - LSV[0])
    while i < 10:
        t = abs(at - LSV[i])
        if t < vel:
            vel = t
            Rvalue = i
        i = i + 1
    
    Rvalue = Rvalue * 0.005
    Rvalue = 1.6 - Rvalue
    return Rvalue
            
def turnright(at):
    v = 0
    Rvalue = 0
    i = 0
    
    if at > RSV[6]:
        v = 1.44
        return v
    
    vel = abs(at - RSV[0])
    while i < 10:
        t = abs(at - RSV[i])
        if t < vel:
            vel = t
            Rvalue = i
        i = i + 1
    
    Rvalue = Rvalue * 0.005
    Rvalue = 1.4 + Rvalue
    return Rvalue

def tleft():
    resetCounts()
    while True:
        pwm.set_pwm(LSERVO, 0, math.floor(1.4 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.4 / 20 * 4096));
        LC = getCounts()
        w,e = LC
        if w > 14:
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
            break
    
def tright():
    resetCounts()
    while True:
        pwm.set_pwm(LSERVO, 0, math.floor(1.6 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.6 / 20 * 4096));
        LC = getCounts()
        w,e = LC        
        if w > 14:
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
            break
    
def front():
    resetCounts()
    X1 = 1.6
    Y1 = 1.4
    DISTANCE = 7.2
    Kp = 0.75
    while True:
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()
        fDistance = fSensor.get_distance()
        fDistance = fDistance/25.3
        rDistance = rDistance/25.3
        lDistance = lDistance/25.3
        
        LC = getCounts()
        w,e = LC
        
        if fDistance < 9:
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
            break
        
       
        ERROR = DISTANCE - lDistance
        DesireSpeed = Kp * ERROR
        
        ERRORr = DISTANCE - rDistance
        DesireSpeedr = Kp * ERROR
        
        if rDistance < DISTANCE:
            X1 = turnleft(DesireSpeedr)
            Y1 = 1.4
        elif lDistance < DISTANCE:
            Y1 = turnright(DesireSpeed)
            X1 = 1.6
        elif rDistance > DISTANCE+2 and rDistance < 18:
            Y1 = turnright(DesireSpeedr)
            X1 = 1.6
        elif lDistance > DISTANCE+2 and lDistance < 18:
            X1 = turnleft(DesireSpeed)
            Y1 = 1.4
        else:
            X1 = 1.6
            Y1 = 1.4
            
        pwm.set_pwm(LSERVO, 0, math.floor(X1 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(Y1 / 20 * 4096));
        
        if w > 70:
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
            break
def gotonext(ans,cur):
    nextn = 0
    nexte = 0
    nextw = 0
    nexts = 0
    cell = 0
    k = 0
    while True:
        if k == 16:
            break
        if maze[k].north == '?':
            nextn = 1
            break
        if maze[k].east == '?':
            nexte = 1
            break
        if maze[k].west == '?':
            nextw = 1
            break
        if maze[k].south == '?':
            nexts = 1
            break
        k = k + 1
    if nextn == 1:
        cell = k-4
    if nexte == 1:
        cell = k+1
    if nextw == 1:
        cell = k-1
    if nexts == 1:
        cell = k+4
    while True:
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()
        
        fDistance = fDistance/25.3
        rDistance = rDistance/25.3
        lDistance = lDistance/25.3
        print("Loop2")
        if ((maze[ans].north == 'O' and maze[ans-4].visited == False) or (maze[ans].east == 'O' and maze[ans+1].visited == False)or (maze[ans].west == 'O' and maze[ans-1].visited == False) or (maze[ans].south == 'O' and maze[ans+4].visited == False)):
            if maze[ans].south == 'O':
                if cur == 'N':
                    tright()
                    tright()
                    cur = 'S'
                elif cur == 'E':
                    tright()
                    cur = 'S'
                elif cur == 'W':
                    tleft()
                    cur = 'S'
                return ans,cur
            if maze[ans].north == 'O':
                if cur =='E':
                    tleft()
                    cur = 'N'
                elif cur == 'W':
                    tright()
                    cur = 'N'
                elif cur == 'S':
                    tright()
                    tright()
                    cur = 'N'
                return ans,cur
            if maze[ans].east == 'O':
                if cur == 'N':
                    tright()
                    cur = 'E'
                elif cur == 'S':
                    tleft()
                    cur = 'E'
                elif cur == 'W':
                    tright()
                    tright()
                    cur = 'E'
                return ans,cur
            if maze[ans].west == 'O':
                tleft()
                if cur == 'N':
                    tleft()
                    cur = 'W'
                elif cur == 'E':
                    tright()
                    tright()
                    cur = 'W'
                elif cur == 'S':
                    tright()
                    cur = 'W'
                return ans,cur
                
        if fDistance < 18 and rDistance < 18 and lDistance < 18:
            tright()
            if cur == 'N':
                cur = 'E'
            elif cur == 'E':
                cur = 'S'
            elif cur == 'S':
                cur = 'W'
            elif cur == 'W':
                cur = 'N'
            continue
        if fDistance > 18:
            front()
            if cur == 'N':
                ans = ans - 4
            elif cur == 'E':
                ans = ans + 1
            elif cur == 'S':
                ans = ans + 4
            elif cur == 'W':
                ans = ans - 1
        if rDistance > 18:
            tright()
            if cur == 'N':
                ans = ans + 1
                cur = 'E'
            elif cur == 'E':
                ans = ans + 4
                cur = 'S'
            elif cur == 'S':
                ans = ans - 1
                cur = 'W'
            elif cur == 'W':
                ans = ans - 4
                cur = 'N'
            
            front()
        if lDistance > 18:
            tleft()
            if cur == 'N':
                ans = ans - 1
                cur = 'W'
            elif cur == 'E':
                ans = ans - 4
                cur = 'N'
            elif cur == 'S':
                ans = ans + 1
                cur = 'E'
            elif cur == 'W':
                ans = ans + 4
                cur = 'S'
            front()
        


def FUNCTION_RANDOM():
    while True:
        f=0
        l=0
        r=0
        c=0
        ran = random.choice([1,2,3])
        ran2 = random.choice([1,2])
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()
        
        fDistance = fDistance/25.3
        rDistance = rDistance/25.3
        lDistance = lDistance/25.3
        
        
        
        if fDistance > 18:
            f = 1
        if rDistance > 18:
            r = 1
        if lDistance > 18:
            l = 1
        if l == 0 and r == 0 and f == 0:
            tright()
            continue
        
        if l ==1 and r == 0 and f == 0:
            tleft()
            front()
        elif r == 1 and l == 0 and f == 0:
            tright()
            front()
        elif f == 1 and r == 0 and l == 0:
            front()
        elif f == 0:
            if ran2 == 1:
                tright()
                front()
            else:
                tleft()
                front()
        elif r == 0:
            if ran2 == 1:
                front()
            else:
                tleft()
                front()
        elif l == 0:
            if ran2 ==1:
                front()
            else:
                tright()
                front()
        else:
            if ran == 1:
                front()
            elif ran == 2:
                tright()
                front()
            else:
                tleft()
                front()
        print("New cell")
        
        time.sleep(1)

def FUNCTION_MAPPING():
    ans = int(input("Please enter the number of cell where the robot will be placed: "))
    ans = ans-1
    cur = 'N'
    count = 0
    while True:
        TEMP = 0
        f=0
        l=0
        r=0
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()
        
        fDistance = fDistance/25.3
        rDistance = rDistance/25.3
        lDistance = lDistance/25.3
        
        maze[ans].visited = True
        count = count +1
        
        print("Current cell: ")
        print(ans)
        
        if cur == 'N':
            
            if fDistance > 18:
                
                maze[ans].north = 'O'
                f = 1
            else:
                maze[ans].north = 'W'
            if rDistance > 18:
                maze[ans].east = 'O'
                r = 1
            else:
                maze[ans].east = 'W'
            if lDistance > 18:
                maze[ans].west = 'O'
                l = 1
            else:
                maze[ans].west = 'W'
        if cur == 'E':
            if fDistance > 18:
                maze[ans].east = 'O'
                f = 1
            else:
                maze[ans].east = 'W'
            if rDistance > 18:
                print("South of the maze")
                maze[ans].south = 'O'
                print(maze[ans].south)
                r = 1
            else:
                maze[ans].south = 'W'
            if lDistance > 18:
                maze[ans].north = 'O'
                l = 1
            else:
                maze[ans].north = 'W'
        if cur == 'W':
            if fDistance > 18:
                maze[ans].west = 'O'
                f = 1
            else:
                maze[ans].west = 'W'
            if rDistance > 18:
                maze[ans].north = 'O'
                r = 1
            else:
                maze[ans].north = 'W'
            if lDistance > 18:
                maze[ans].south = 'O'
                l = 1
            else:
                maze[ans].south = 'W'
        if cur == 'S':
            if fDistance > 18:
                maze[ans].south = 'O'
                f = 1
            else:
                maze[ans].south = 'W'
            if rDistance > 18:
                maze[ans].west = 'O'
                r = 1
            else:
                maze[ans].west = 'W'
            if lDistance > 18:
                maze[ans].east = 'O'
                l = 1
            else:
                maze[ans].east = 'W'
                
        if maze[ans].north == 'O' and ans+1 > 4:
            maze[ans-4].south = 'O'
        elif maze[ans].north == 'W' and ans+1 > 4:
            maze[ans-4].south = 'W'
        if maze[ans].east == 'O' and (ans+1) % 4 != 0:
            maze[ans+1].west = 'O'
        elif maze[ans].east == 'W'and (ans+1) % 4 != 0:
            maze[ans+1].west = 'W'
        if maze[ans].west == 'O'and (ans+1) % 4 != 1:
            maze[ans-1].east = 'O'
        elif maze[ans].west == 'W'and (ans+1) % 4 != 1:
            maze[ans-1].east = 'W'
        if maze[ans].south == 'O'and ans+1 < 13:
            maze[ans+4].north = 'O'
        elif maze[ans].south == 'W' and ans+1 < 13:
            maze[ans+4].north = 'W'
        
        #a = Cell('?','?','?','?',False) 
        #a = maze[ans]
        print(maze[ans].south)
        if maze[ans].east == 'O':
            if cur == 'N' and maze[ans+1].visited == False:
                tright()
                front()
                cur = 'E'
                ans = ans+1
                TEMP = 1
            elif cur == 'E' and maze[ans+1].visited == False:
                front()
                ans = ans+1
                TEMP = 1
            elif cur == 'W' and maze[ans+1].visited == False:
                tright()
                tright()
                front()
                cur = 'E'
                ans = ans+1
                TEMP = 1
            elif cur == 'S' and maze[ans+1].visited == False:
                tleft()
                front()
                cur = 'E'
                ans = ans+1
                TEMP = 1
        if maze[ans].north == 'O':
            if cur == 'N'and maze[ans-4].visited == False:
                front()
                ans = ans-4
                TEMP = 1
            elif cur == 'E'and maze[ans-4].visited == False:
                tleft()
                front()
                cur = 'N'
                ans = ans-4
                TEMP = 1
            elif cur == 'W' and maze[ans-4].visited == False:
                tright()
                front()
                cur = 'N'
                ans = ans-4
                TEMP = 1
            elif cur == 'S' and maze[ans-4].visited == False:
                tright()
                tright()
                front()
                cur = 'N'
                ans = ans-4
                TEMP = 1
        if maze[ans].west == 'O':
            
            if cur == 'N' and maze[ans-1].visited == False:
                tleft()
                front()
                cur = 'W'
                ans = ans-1
                TEMP = 1
            elif cur == 'E' and maze[ans-1].visited == False:
                tleft()
                tleft()
                front()
                cur = 'W'
                ans = ans-1
                TEMP = 1
            elif cur == 'W' and maze[ans-1].visited == False:
                front()
                ans = ans-1
                TEMP = 1
            elif cur == 'S' and maze[ans-1].visited == False:
                tright()
                front()
                cur = 'W'
                ans = ans-1
                TEMP = 1
        if maze[ans].south == 'O':
            
            print("Entered")
            if cur == 'S' and maze[ans+4].visited == False:
                print("First")
                front()
                ans = ans+4
                TEMP = 1
            elif cur == 'N' and maze[ans+4].visited == False:
                print("Second")
                tright()
                tright()
                front()
                ans = ans+4
                cur = 'S'
                TEMP = 1
            elif cur == 'E' and maze[ans+4].visited == False:
                print("Third")
                tright()
                front()
                ans = ans+4
                cur = 'S'
                TEMP = 1
            elif cur == 'W' and maze[ans+4].visited == False:
                print("Fourth")
                tleft()
                front()
                ans = ans+4
                cur = 'S'
                TEMP = 1
        if count == 16:
            break
        printMaze(maze)
        
        if TEMP == 0 :
            abc = gotonext(ans,cur)
            ans,cur=abc
            print(ans,cur)
            continue
M = [Cell('?','?','?','?', False),Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False),
        Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False),
        Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False),
        Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False)]        
def FUNCTION_PATH():
    
    ans = int(input("Please enter the number of cell where the robot will start: "))
    end = int(input("Please enter the number of cell where the robot will stop: "))
    ans = ans-1
    cur = 'N'
    count = 0
    while True:
        if ans == end-1:
            break
        
        TEMP = 0
        f=0
        l=0
        r=0
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()
        
        fDistance = fDistance/25.3
        rDistance = rDistance/25.3
        lDistance = lDistance/25.3
        
        M[ans].visited = True
        count = count +1
        
        print("Current cell: ")
        print(ans)
        
        if cur == 'N':
            
            if fDistance > 18:
                
                M[ans].north = 'O'
                f = 1
            else:
                M[ans].north = 'W'
            if rDistance > 18:
                M[ans].east = 'O'
                r = 1
            else:
                M[ans].east = 'W'
            if lDistance > 18:
                M[ans].west = 'O'
                l = 1
            else:
                M[ans].west = 'W'
        if cur == 'E':
            if fDistance > 18:
                M[ans].east = 'O'
                f = 1
            else:
                M[ans].east = 'W'
            if rDistance > 18:
                print("South of the M")
                M[ans].south = 'O'
                print(M[ans].south)
                r = 1
            else:
                M[ans].south = 'W'
            if lDistance > 18:
                M[ans].north = 'O'
                l = 1
            else:
                M[ans].north = 'W'
        if cur == 'W':
            if fDistance > 18:
                M[ans].west = 'O'
                f = 1
            else:
                M[ans].west = 'W'
            if rDistance > 18:
                M[ans].north = 'O'
                r = 1
            else:
                M[ans].north = 'W'
            if lDistance > 18:
                M[ans].south = 'O'
                l = 1
            else:
                M[ans].south = 'W'
        if cur == 'S':
            if fDistance > 18:
                M[ans].south = 'O'
                f = 1
            else:
                M[ans].south = 'W'
            if rDistance > 18:
                M[ans].west = 'O'
                r = 1
            else:
                M[ans].west = 'W'
            if lDistance > 18:
                M[ans].east = 'O'
                l = 1
            else:
                M[ans].east = 'W'
                
        if M[ans].north == 'O' and ans+1 > 4:
            M[ans-4].south = 'O'
        elif M[ans].north == 'W' and ans+1 > 4:
            M[ans-4].south = 'W'
        if M[ans].east == 'O' and (ans+1) % 4 != 0:
            M[ans+1].west = 'O'
        elif M[ans].east == 'W'and (ans+1) % 4 != 0:
            M[ans+1].west = 'W'
        if M[ans].west == 'O'and (ans+1) % 4 != 1:
            M[ans-1].east = 'O'
        elif M[ans].west == 'W'and (ans+1) % 4 != 1:
            M[ans-1].east = 'W'
        if M[ans].south == 'O'and ans+1 < 13:
            M[ans+4].north = 'O'
        elif M[ans].south == 'W' and ans+1 < 13:
            M[ans+4].north = 'W'
        
        #a = Cell('?','?','?','?',False) 
        #a = M[ans]
        print(M[ans].south)
        if M[ans].east == 'O':
            if cur == 'N' and M[ans+1].visited == False:
                tright()
                front()
                cur = 'E'
                ans = ans+1
                TEMP = 1
            elif cur == 'E' and M[ans+1].visited == False:
                front()
                ans = ans+1
                TEMP = 1
            elif cur == 'W' and M[ans+1].visited == False:
                tright()
                tright()
                front()
                cur = 'E'
                ans = ans+1
                TEMP = 1
            elif cur == 'S' and M[ans+1].visited == False:
                tleft()
                front()
                cur = 'E'
                ans = ans+1
                TEMP = 1
        if M[ans].north == 'O':
            if cur == 'N'and M[ans-4].visited == False:
                front()
                ans = ans-4
                TEMP = 1
            elif cur == 'E'and M[ans-4].visited == False:
                tleft()
                front()
                cur = 'N'
                ans = ans-4
                TEMP = 1
            elif cur == 'W' and M[ans-4].visited == False:
                tright()
                front()
                cur = 'N'
                ans = ans-4
                TEMP = 1
            elif cur == 'S' and M[ans-4].visited == False:
                tright()
                tright()
                front()
                cur = 'N'
                ans = ans-4
                TEMP = 1
        if M[ans].west == 'O':
            
            if cur == 'N' and M[ans-1].visited == False:
                tleft()
                front()
                cur = 'W'
                ans = ans-1
                TEMP = 1
            elif cur == 'E' and M[ans-1].visited == False:
                tleft()
                tleft()
                front()
                cur = 'W'
                ans = ans-1
                TEMP = 1
            elif cur == 'W' and M[ans-1].visited == False:
                front()
                ans = ans-1
                TEMP = 1
            elif cur == 'S' and M[ans-1].visited == False:
                tright()
                front()
                cur = 'W'
                ans = ans-1
                TEMP = 1
        if M[ans].south == 'O':
            
            print("Entered")
            if cur == 'S' and M[ans+4].visited == False:
                print("First")
                front()
                ans = ans+4
                TEMP = 1
            elif cur == 'N' and M[ans+4].visited == False:
                print("Second")
                tright()
                tright()
                front()
                ans = ans+4
                cur = 'S'
                TEMP = 1
            elif cur == 'E' and M[ans+4].visited == False:
                print("Third")
                tright()
                front()
                ans = ans+4
                cur = 'S'
                TEMP = 1
            elif cur == 'W' and M[ans+4].visited == False:
                print("Fourth")
                tleft()
                front()
                ans = ans+4
                cur = 'S'
                TEMP = 1
        
        
        if count == 16:
            break
        printMaze(M)
        
        if TEMP == 0 :
            abc = gotonext2(ans,cur)
            ans,cur=abc
            print(ans,cur)
            continue
def gotonext2(ans,cur):
    nextn = 0
    nexte = 0
    nextw = 0
    nexts = 0
    cell = 0
    k = 0
    while True:
        if k == 16:
            break
        if M[k].north == '?':
            nextn = 1
            break
        if M[k].east == '?':
            nexte = 1
            break
        if M[k].west == '?':
            nextw = 1
            break
        if M[k].south == '?':
            nexts = 1
            break
        k = k + 1
    if nextn == 1:
        cell = k-4
    if nexte == 1:
        cell = k+1
    if nextw == 1:
        cell = k-1
    if nexts == 1:
        cell = k+4
    while True:
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()
        
        fDistance = fDistance/25.3
        rDistance = rDistance/25.3
        lDistance = lDistance/25.3
        print("Loop2")
        if ((M[ans].north == 'O' and M[ans-4].visited == False) or (M[ans].east == 'O' and M[ans+1].visited == False)or (M[ans].west == 'O' and M[ans-1].visited == False) or (M[ans].south == 'O' and M[ans+4].visited == False)):
            if M[ans].south == 'O':
                if cur == 'N':
                    tright()
                    tright()
                    cur = 'S'
                elif cur == 'E':
                    tright()
                    cur = 'S'
                elif cur == 'W':
                    tleft()
                    cur = 'S'
                return ans,cur
            if M[ans].north == 'O':
                if cur =='E':
                    tleft()
                    cur = 'N'
                elif cur == 'W':
                    tright()
                    cur = 'N'
                elif cur == 'S':
                    tright()
                    tright()
                    cur = 'N'
                return ans,cur
            if M[ans].east == 'O':
                if cur == 'N':
                    tright()
                    cur = 'E'
                elif cur == 'S':
                    tleft()
                    cur = 'E'
                elif cur == 'W':
                    tright()
                    tright()
                    cur = 'E'
                return ans,cur
            if M[ans].west == 'O':
                tleft()
                if cur == 'N':
                    tleft()
                    cur = 'W'
                elif cur == 'E':
                    tright()
                    tright()
                    cur = 'W'
                elif cur == 'S':
                    tright()
                    cur = 'W'
                return ans,cur
                
        if fDistance < 18 and rDistance < 18 and lDistance < 18:
            tright()
            if cur == 'N':
                cur = 'E'
            elif cur == 'E':
                cur = 'S'
            elif cur == 'S':
                cur = 'W'
            elif cur == 'W':
                cur = 'N'
            continue
        if fDistance > 18:
            front()
            if cur == 'N':
                ans = ans - 4
            elif cur == 'E':
                ans = ans + 1
            elif cur == 'S':
                ans = ans + 4
            elif cur == 'W':
                ans = ans - 1
        if rDistance > 18:
            tright()
            if cur == 'N':
                ans = ans + 1
                cur = 'E'
            elif cur == 'E':
                ans = ans + 4
                cur = 'S'
            elif cur == 'S':
                ans = ans - 1
                cur = 'W'
            elif cur == 'W':
                ans = ans - 4
                cur = 'N'
            
            front()
        if lDistance > 18:
            tleft()
            if cur == 'N':
                ans = ans - 1
                cur = 'W'
            elif cur == 'E':
                ans = ans - 4
                cur = 'N'
            elif cur == 'S':
                ans = ans + 1
                cur = 'E'
            elif cur == 'W':
                ans = ans + 4
                cur = 'S'
            front()
while True:
    print("Menu")
    print("1-Calibration Menu")
    print("2-LocalizationMenu")
    print("3-Mapping Menu")
    print("4-Path Planning Menu")
    answer = int(input())
    if answer == 1:
        calibrateSpeeds()
    if answer == 2:
        FUNCTION_RANDOM()
    if answer == 3:
        FUNCTION_MAPPING()
    if answer == 4:
        FUNCTION_PATH()    