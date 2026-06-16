#! /usr/bin/python
'''The main python file, it contains the code for both robot runs and helper functions'''
from dataclasses import dataclass
import json
from math import acos, asin, cos, degrees, pi, atan, ceil, copysign, floor, sin, sqrt, tan
import os
import multiprocessing as mp
import random
import threading
from time import sleep, time
from log import *
from utilities import func_thread
from sys import argv

import cv2
from picamera2 import Picamera2

run_type = None
#full_wro.service

def assignArgs(args: list) -> None:
    global run_type
    for i in args:
        match i:
            case "-obs":
                run_type = "Obstacle"
            case "-ope":
                run_type = "Open"
            case "-tst":
                run_type = "Test"
            case "-2nd":
                run_type = "2nd day"
            case _:
                log.error(f"System argument {i} is not recognized")
                log.save_logs_to_file()
                log.save_temp_logs_to_file()
             

if len(argv) > 1:
    assignArgs(argv[1:])
    with open(directory / "args.txt","w") as file:
        for i in argv[1:]:
            file.write(i+"\n")
else:
    with open(directory / "args.txt","r") as file:
        args = file.readlines()
        args = [i.replace("\n","") for i in args]
    assignArgs(args)


TURNTARGET_INNER=0
TURNTARGET_INNER_OBS=1
TURNTARGET_MIDDLE=2
TURNTARGET_OUTER_OBS=3
TURNTARGET_OUTER=4
TURN_RADIUS=12
turnDists=[98, 74, 64, 54, 34] #94->98
laneWallDists=[20, 40, 50, 60, 80]
'''Indexed by lane (left/right). 0:left,..,4:right'''
def angularToXy(angle:int, distance:int):
    '''Helper method to convert angular (angle distance) coordinates into x and y coordinates'''
    x=distance*sin(angle/180*pi)
    y=distance*cos(angle/180*pi)
    return x,y

def XyToAngular(x:int,y:int):
    '''Helper method to convert x and y coordinates into angle and distance'''
    if x==0 or y==0: return 0, y
    angle=atan(x/y)
    distance=x/sin(angle)
    angle=angle/pi*180
    return angle,distance
@dataclass
class Object:
    '''Dataclass representing a detected object'''
    objStart:tuple[float,float]
    '''The leftmost point of the object'''
    objEnd:tuple[float,float]
    '''The rightmost point of the object'''
    relToAbsX:int=-1
    relToAbsY:int=-1
    angle:float=-1
    '''The rotation of the object (relative to the walls)'''
    def getCenter(self)->tuple[int,int]:
        '''Getter method of 'center' property
        
        Returns the center of the detected object (average of objStart and objEnd)'''
        return ((self.objStart[0]+self.objEnd[0])/2,(self.objStart[1]+self.objEnd[1])/2)
    center=property(getCenter,None,None)
    '''Returns the center of the detected object (average of objStart and objEnd)'''
    def isEmpty(self)->bool:
        '''Getter method of 'empty' property
        
        True if object was not found'''
        return True if self.objStart[0]==-1 else False
    empty=property(isEmpty,None,None)
    '''True if object was not found'''
    
    def toAbsolute(self):
        '''Returns this object converted to absolute position'''
        objStartAbs=[self.objStart[0]+self.relToAbsX,self.objStart[1]+self.relToAbsY]
        objEndAbs=[self.objEnd[0]+self.relToAbsX,self.objEnd[1]+self.relToAbsY]
        return Object(objStartAbs,objEndAbs)
    def toRelAngle(self):
        relStartAngle,startDist=XyToAngular(*self.objStart)
        relEndAngle,endDist=XyToAngular(*self.objEnd)
        relStartAngle-=self.angle
        relEndAngle-=self.angle
        return Object(angularToXy(relStartAngle,startDist),angularToXy(relEndAngle,endDist),angle=0)
    def toRelative(self):
        '''Returns this object converted to relative position'''
        objStartRel=[self.objStart[0]-self.relToAbsX,self.objStart[1]-self.relToAbsY]
        objEndRel=[self.objEnd[0]-self.relToAbsX,self.objEnd[1]-self.relToAbsY]
        return Object(objStartRel,objEndRel)
    
CAMERA_ANGLE_OFFSET=-1.5 #how much (degrees) the camera is rotated to the left relative to the robot
HORIZONTAL_FOV=53.1
VERTICAL_FOV=41.49
CAMERA_VERTICAL_TILT=9.5 #10.76
cameraPos=(0,-4,19.5)
GREEN=1
'''Constant for the color Green. Used for obstacle color detection and storing.'''
RED=2
'''Constant for the color Red. Used for obstacle color detection and storing.'''
RMG_BORDER=0 #red-green

ANALYSE_NTH_PIXELS=4
'''Analyse every this many pixel'''

def mapPointToCam(point:tuple[int,int,int])-> tuple:
    log.debug("mapPointToCam %s %s %s"%point)
    px=320+(degrees(atan((point[0])/(point[1]-cameraPos[1])))+CAMERA_ANGLE_OFFSET)*(640/HORIZONTAL_FOV)
    # print(degrees(atan((point[2]-cameraPos[2])/(point[1]-cameraPos[1]))))
    # print((point[2]-cameraPos[2]))
    # print((point[1]-cameraPos[1]))
    # print(degrees(atan((point[2]-cameraPos[2])/(point[1]-cameraPos[1]))))
    # py=240-(degrees(atan((point[2]-cameraPos[2])/(point[1]-cameraPos[1])))+11.5)*(480/44)
    py=240-(degrees(atan((point[2]-cameraPos[2])/(point[1]-cameraPos[1])))+CAMERA_VERTICAL_TILT)*(480/VERTICAL_FOV)
    if px<0: px=0
    if px>=640: px=639
    if py<0: py=0
    if py>=480: py=479
    return (px,py)
camQueue = mp.Queue(10)

def camProcess(queue) -> None:
    cam = Picamera2()

    mode = cam.sensor_modes[1]
    log.debug(cam.sensor_modes)
    config = cam.create_preview_configuration(sensor={'output_size': mode['size'], 'bit_depth':
    mode['bit_depth']})
    cam.configure(config)

    open(directory / "logs/img.stormslog","w").close()

    cam.start()
    ptr=0
    while True:
        data:Object|str = queue.get()
        if data == "close":
            cam.close()
            break
        #epic calculation
        #data: topleft x,y botright x,y
        obj=data.toRelAngle()
        topLeft=(*obj.objStart,10)
        botRight=(*obj.objEnd,0)
        ptr+=1
        cam.capture_file("logs/imgs/snapshot%s.png"%ptr)
        img = cv2.imread("logs/imgs/snapshot%s.png"%ptr)
        imgTopLeft=mapPointToCam(topLeft) #0,1
        imgTopLeft=(max(0,imgTopLeft[0]-5),imgTopLeft[1])
        imgBotRight=mapPointToCam(botRight) #2,3
        imgBotRight=(min(639,imgBotRight[0]+5),imgBotRight[1])
        imgCut = img[round(imgTopLeft[1]):round(imgBotRight[1]),round(imgTopLeft[0]):round(imgBotRight[0])]

        color=cv2.mean(imgCut)
        rmg=color[2]-color[1] #BGR


        # rect=cv2.rectangle(img,(int(imgTopLeft[0]),int(imgTopLeft[1])),(int(imgBotRight[0]),int(imgBotRight[1])),(255,0,0),2)
        # rect=cv2.putText(rect,"rmg: %s"%rmg,(int(imgTopLeft[0])+10,int(imgTopLeft[1])-30),cv2.FONT_HERSHEY_COMPLEX,1,(200,0,0))
        # cv2.imwrite(("rect%s.png"%ptr),rect)
        queue.put((rmg,color[0]))

        data = {
            "name": f"snapshot{ptr}.png",
            "points": [imgTopLeft,imgBotRight],
            "rmg": rmg,
            "id": ptr,
            "time": time()
        }

        jdata = json.dumps(data)

        with open(directory / "logs/img.stormslogtemp","w") as f:
            f.write(jdata)
            f.flush()
        
        with open(directory / "logs/img.stormslog","a") as f:
            f.write(jdata)
            f.write("\n")
            f.flush()

    #stop camera

camP = mp.Process(target=camProcess, args=(camQueue,))
os.environ["LIBCAMERA_LOG_LEVELS"] = "*:3"
camP.start()
def closeCam():
    camQueue.put("close")
    camP.join()

def detectObjectColor(obj:Object)->int:
    '''Checks the color of the object using the RaspberryPi camera by communicating with the other core'''
    data=obj
    t0=time.time()
    camQueue.put(data)
    rmg,blue=camQueue.get()
    #log.debug("topleft %s br %s imgtl %s imgbr %s"%(topLeft,botRight,imgTopLeft,imgBotRight))
    rmg=int(rmg)
    log.debug("RMG: %s blue %s"%(rmg,blue))
    log.debug("dtime: %s"%(time.time()-t0))
    if rmg>RMG_BORDER:
        col=RED
    else: col=GREEN
    log.info(col)
    return col
from LedAndKey import *
from ESP32_Service import packetCount
from ESP32_Service import *
import LidarService
from LidarService import *
from Buzzer import *
import time
# distSensor=VL53L1X()
# '''Laser distance sensor behind the robot'''

TICKS_PER_CM=37.12*(1/3) #faster motor
'''Convert internal ticks to CM. (Motor rotation is measured in ticks by the ESP)'''

direction:int=1
'''Stores the detected randomized direction during obstacle challenge. 
1: robot turns right
-1: robot turns left'''


# TODO: Pixy camera currently not in use

colorMatrixPrequel = [RED,GREEN]
getColorCounter = 1


CAM_WIDTH=315
CAM_HEIGHT=207
CAM_RIGHT_CUTOFF=CAM_WIDTH-0 #200
CAM_LEFT_CUTOFF=50
CAM_BOT_CUTOFF=CAM_HEIGHT-10
CAM_TOP_CUTOFF=0
def isInsideCamArea(x,y):
    return True #currently camera cutoff is not in use
    return x<CAM_RIGHT_CUTOFF and x>CAM_LEFT_CUTOFF and y>CAM_TOP_CUTOFF and y<CAM_BOT_CUTOFF

defaultColor=RED
''''''
def checkColor()->int:
    '''Checks the color of the closest obstacle outside the cutoff area using the Pixy camera.'''
    global getColorCounter
    global CAM_RIGHT_CUTOFF
    global CAM_LEFT_CUTOFF
    global pixyObjects
    if lane==0:
        CAM_LEFT_CUTOFF=90
        CAM_RIGHT_CUTOFF=240
    elif lane>0:
        CAM_LEFT_CUTOFF=0
        CAM_RIGHT_CUTOFF=CAM_WIDTH-150
    elif lane<0:
        CAM_LEFT_CUTOFF=150
        CAM_RIGHT_CUTOFF=CAM_WIDTH
    # getColorCounter += 1
    # pixyObjects.append((110+random.randint(0,100),120+random.randint(-10,10),10+random.randint(-2,5),20+random.randint(-2,5),colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)],True))
    # log.info("color: %s"%colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)])
    # return colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)]
    temp_pixyObjects=[]
    blocks=pixy.BlockArray(4)
    n=pixy.ccc_get_blocks(4,blocks)
    log.info("detected %s blocks"%n)
    if n==0:
        log.error("No blocks detected!")
        pixyObjects=temp_pixyObjects
        return defaultColor
    for b in range(n): temp_pixyObjects.append((blocks[b].m_x,blocks[b].m_y,blocks[b].m_width,blocks[b].m_height,blocks[b].m_signature,False))
    for i in range(n):
        b:pixy.Block=blocks[i]
        log.debug("detected at %s %s, size %s %s with signature: %s"%(b.m_x,b.m_y,b.m_width,b.m_height,b.m_signature))
        if isInsideCamArea(b.m_x,b.m_y):
            log.info("i %s"%i)
            detected=list(temp_pixyObjects[i])
            detected[5]=True
            temp_pixyObjects[i]=tuple(detected)
            log.info("Final color %s"% "GREEN" if b.m_signature==GREEN else "RED")
            pixyObjects=temp_pixyObjects
            return b.m_signature
    else:
        log.error("All blocks outside cutoff range!! cutoffs: left %s right %s top %s bot %s"%(CAM_LEFT_CUTOFF,CAM_RIGHT_CUTOFF,CAM_TOP_CUTOFF,CAM_BOT_CUTOFF))
        pixyObjects=temp_pixyObjects
        return defaultColor


wIntegral=0
'''Wall distance following integral variable'''
wLastError=0
'''Wall distance following last error variable for derivative'''
wkP=1
'''Wall distance following proportional constant'''
wkI=0.1
'''Wall distance following integral constant'''
wkD=0.5
'''Wall distance following derivative constant'''
wallTarget=50
'''Wall distance following target distance in centimeters'''
pilotHeadingTarget:int=0
'''Target robot direction'''
lastHeading=0
'''Last heading direction recorded'''
heading0=0
'''0 heading offset'''

lastTOFDist=0

SGYRO_LIMITED=20
'''Limited sMode gyro steer turning'''
SGYRO_NORMAL=50
'''Normal sMode gyro steer turning'''

PILOT_NONE=0
'''Robot piloting mode, no piloting'''
PILOT_FOLLOW_LEFT=1
'''Robot piloting mode, follow left wall at wallTarget'''
PILOT_FOLLOW_RIGHT=2
'''Robot piloting mode, follow right wall at walltarget'''

pilotMode:int=0
'''Current pilot mode'''

def setPilotMode(pilotModeIn:int, param:int=10000):
    '''Set pilotMode and wallTarget if needed'''
    global pilotMode
    global wallTarget
    pilotMode=pilotModeIn
    if pilotModeIn==PILOT_FOLLOW_LEFT or pilotModeIn==PILOT_FOLLOW_RIGHT: wallTarget=param

def go(headingTarget:int, backwards=False, speed_override:int=None, steerMode:int=SMODE_GYRO):
    '''Starts the robot with given parameters

    speed_override: speed of the robot measured in ticks/second. (0-3000) If None, tempo is used<br>
    headingTarget: Target direction in degrees (-90 - 90)<br>
    pilotMode: If left at default value no piloting, otherwise set given pilotmode (PILOT_FOLLOW_LEFT, PILOT_FOLLOW_RIGHT)<br>
    wallDistance: Only relevant if pilotMode is not PILOT_NONE, sets target wall distance for wall following'''
    global pilotHeadingTarget
    log.debug("go with speed_override: %s headingT: %s "%(speed_override,headingTarget))
    #reset wall following integral variable to ensure no previous buildup is kept
    if speed_override!=None: setTargetSpeed(speed_override)
    else: setTargetSpeed(tempo if not backwards else -tempo)
    pilotHeadingTarget=headingTarget
    setHeadingTarget(headingTarget)
    
    setSteerMode(steerMode)
    if not backwards:
        setVMode(VMODE_FORWARD)
    else:
        setVMode(VMODE_BACKWARD)
    log.debug("go over")

def goUnreg(power:int,headingTarget:int):
    '''Starts the motors without PID speed control, instead using constant power

    power: The constant power (-100 - 100)<br>
    headingTarget: Target direction in degrees (-90 - 90)'''
    global pilotHeadingTarget
    setUnregulatedPower(power)
    pilotHeadingTarget=headingTarget
    setHeadingTarget(headingTarget)
    setSteerMode(SMODE_GYRO)
    setVMode(VMODE_UNREGULATED)



def isInsideRect(botLeft,topRight,point):
    '''Checks whether given point (x,y) is inside rectangle defined by point bottom left (x,y) and top right (x,y)'''
    return point[0]>=botLeft[0] and point[0]<=topRight[0] and point[1]>=botLeft[1] and point[1]<=topRight[1]

WARNING_DZ_TOLERANCE=5
'''Lidar deadzone tolerance, robot won't signal an error message if angle is within tolerance of deadzone'''
def readLidar(degree)->int:
    '''Returns the lidar distance at the given angle from the stored array of angles. Also avoids the lidar deadzone'''
    global lidarDOI
    degree*=-1 #lidar is upside down
    degree+=LIDAR_ANGLE_OFFSET #lidar is not perfectly straight
    degree=(degree+3600)%360
    
    if degree>180:
        degree-=360
    originalDegree=degree
    if degree<LIDAR_DEADZONE_START-WARNING_DZ_TOLERANCE or degree>LIDAR_DEADZONE_END+WARNING_DZ_TOLERANCE:
        log.warning("lidar danger zone! req. degree: %s, heading: %s" % (degree,getHeading()))
    if degree<LIDAR_DEADZONE_START:
        degree=LIDAR_DEADZONE_START
    if degree>LIDAR_DEADZONE_END:
        degree=LIDAR_DEADZONE_END
    distance=DISTANCE_MAP[((int(degree-90)+360*100))%360]
    if degree!=originalDegree:
        
        corrDistance=cos(abs(degree-originalDegree)/180*pi)*distance
        log.debug("dz corrected og degree %s corrected degree %s calc angle %s og dist %s corrDist %s"%(originalDegree, degree, abs(degree-originalDegree), distance, corrDistance))
        distance=corrDistance
    lidarDOI.append(degree)
    return distance

def readAbsLidar(degree)->int:
    '''Returns the lidar distance at the given absolute angle (not relative to the robot)'''
    # log.info("h%s"%getHeading())
    return readLidar((degree-getHeading()))

def getAbsX(outer=True)->int:
    '''Returns the robot distance from the outer wall'''
    if direction==(1 if outer else -1):
        return readAbsLidar(-90)
    else:
        return 100-readAbsLidar(90)

def getAbsY(back=False)->int:
    '''Returns the robot distance from the back (behind the robot) wall'''
    # return 300-readAbsLidar(0)
    if back: return readLidarBehind()
    else: return 300-readAbsLidar(0)

def findNearestPointAbs(botLeft:tuple,topRight:tuple,back=True,returnRelative:bool=False):
    '''Finds the nearest point inside defined rectangle (bottom left and top right) relative to the bottom left corner of the section'''
    point= findNearestPoint( (botLeft[0]-getAbsX(), botLeft[1]-getAbsY(back)), (topRight[0]-getAbsX(), topRight[1]-getAbsY(back)))
    if point[1]!=-1:
        if returnRelative:
            return point
        else:
            return (point[0]+getAbsX() , point[1]+getAbsY(back))
    else:
        return (-1,-1)
    
def findNearestPoint(botLeft:tuple,topRight:tuple):
    '''Returns the nearest point inside defined rectangle (bottom left and top right) relative to the lidar'''
    global lidarRects
    lidarRects.append((botLeft,topRight))
    topLeftAngular=XyToAngular(botLeft[0],topRight[1])
    botLeftAngular=XyToAngular(botLeft[0],botLeft[1])
    topRightAngular=XyToAngular(topRight[0],topRight[1])
    botRightAngular=XyToAngular(topRight[0],botLeft[1])
    startAngle=min(topLeftAngular[0],botLeftAngular[0])
    endAngle=max(botRightAngular[0],topRightAngular[0])
    nearestPoint=(1000000,10000000)
    for i in range(floor(startAngle),ceil(endAngle)):
        distance=readAbsLidar(i)
        x,y=angularToXy(i,distance)
        if isInsideRect(botLeft,topRight,(x,y)) and y<nearestPoint[1]: nearestPoint=(x,y)
    if nearestPoint[1]==10000000: nearestPoint=(-1,-1)
    return nearestPoint

FNO_TOLERANCE_CM=10 #how far (vertically) two points have to be to be considered different objects

def findNearestObject(botLeft:tuple,topRight:tuple):
    '''Finds the nearest point inside defined rectangle (bottom left and top right) relative to the lidar
    
    Returns: object leftmost point(x,y), rightmost point (x,y), startAngle, endAngle, startDistance, endDistance'''
    global lidarRects
    log.debug("fno %s %s"%(botLeft,topRight))
    lidarRects.append((botLeft,topRight))
    topLeftAngular=XyToAngular(botLeft[0],topRight[1])
    botLeftAngular=XyToAngular(botLeft[0],botLeft[1])
    topRightAngular=XyToAngular(topRight[0],topRight[1])
    botRightAngular=XyToAngular(topRight[0],botLeft[1])
    startAngle=min(topLeftAngular[0],botLeftAngular[0])
    endAngle=max(botRightAngular[0],topRightAngular[0])
    nearestPoint=(1000000,10000000)
    objStartAngle=-1
    objEndAngle=-1
    lookingForEnd=False
    rightmostPoint=(-1,-1)
    lastPoint=(-1,-1)
    startDist=-1
    endDist=-1
    lastEndDist=-1
    log.info("loop from %s to %s"%(startAngle,endAngle))
    for i in range(floor(startAngle),ceil(endAngle)):
        distance=readAbsLidar(i)
        
        x,y=angularToXy(i,distance)
        if y<nearestPoint[1]-FNO_TOLERANCE_CM and isInsideRect(botLeft,topRight,(x,y)): 
            # objStartAngle=i
            # startDist=distance
            # lookingForEnd=True
            nearestPoint=(x,y)
            log.warn("set start to %s"%i)
        # if y>=botLeft[1]:
        #     log.warn("xyi %s %s %s"%(x,y,i))
        #     # if y>lastPoint[1]+FNO_TOLERANCE_CM:
        #     #     if lookingForEnd:
        #     #         objEndAngle=i-1
        #     #         endDist=lastEndDist
                    
        #     #         rightmostPoint=lastPoint
        #     #         log.warn("set end to %s"%i)
        #     #         lookingForEnd=False
            
            
        #     lastEndDist=distance
        #     lastPoint=(x,y)
        # else:
        #     log.debug("object in front! angle %s dist %s"%(i,distance))
    # if objEndAngle<objStartAngle:
    #     objEndAngle=endAngle
    #     endDist=distance
    #     rightmostPoint=angularToXy(endAngle,distance)
    rightmostPoint=(nearestPoint[0]+5,nearestPoint[1])
    log.warn("right %s left %s"%(rightmostPoint,nearestPoint))
    if nearestPoint[0]==1000000: nearestPoint=(-1,-1)
    #correct for lidar inaccuracy
    # nearestPoint=(nearestPoint[0]-2,nearestPoint[1])
    # rightmostPoint=(rightmostPoint[0]+2,rightmostPoint[1])
    return Object(nearestPoint,rightmostPoint,angle=getHeading())

def findNearestObjectAbs(botLeft:tuple,topRight:tuple, back:bool=True, outer:bool=True, overrideAbsX=None, overrideAbsY=None):
    '''Finds the nearest object inside defined rectangle (bottom left and top right) relative to the bottom left corner of the section
    '''
    log.debug(f"fnoA: overrideX: {overrideAbsX}, overrideX: {overrideAbsY}")
    absX=getAbsX(outer) if overrideAbsX==None else overrideAbsX
    absY=getAbsY(back) if overrideAbsY==None else overrideAbsY
    obj= findNearestObject( (botLeft[0]-absX, botLeft[1]-absY), (topRight[0]-absX, topRight[1]-absY))
    obj.relToAbsX=absX
    obj.relToAbsY=absY
    return obj
# LIDAR_TOF_DIST=21
# '''Distance between ultrasonic sensor and lidar'''
# TOFactive=True
# @func_thread()
# def tofLoop():
#     while True:
#         if TOFactive:
#             updTOF()
#         sleep(0.1)
# # def updTOF():
# #     '''Update stored laser time-of-flight sensor reading'''
# #     global lastTOFDist
# #     # t0=time.time()
# #     dist=distSensor.get_distance()/10+LIDAR_TOF_DIST
# #     if dist<200: lastTOFDist=dist
# #     # log.info('dtime: %s'%(time.time()-t0))

def readLidarBehind()->float:
    '''Returns last stored lidar sensor reading at 180° degrees'''
    global lidarDOI
    lidarDOI.append(182)
    return DISTANCE_MAP[92]
    for i in range(10):
        posI=DISTANCE_MAP[90-LIDAR_ANGLE_OFFSET+i]
        negI=DISTANCE_MAP[90-LIDAR_ANGLE_OFFSET-i]
        if posI!=0 and posI>12:
            lidarDOI.append(180+LIDAR_ANGLE_OFFSET-i)
            return posI
        if negI!=0 and negI>12:
            lidarDOI.append(180+LIDAR_ANGLE_OFFSET-i)
            return negI

    return DISTANCE_MAP[90] #behind the robot

def waitLidarBehind(cm,decreasing=True):
    '''Wait until lidar sensor detects distance smaller (or larger) than cm behind the robot'''
    log.info("waitLB cm %s decreasing? %s 0: %s"%(cm,decreasing,readLidarBehind()))
    condition=True
    sleep(0.1)
    while condition:
        dist=readLidarBehind()
        if decreasing: condition=(cm<dist)
        else: condition=(cm>dist)
        sleep(0.01)
    log.info("waitTof out at %s"%readLidarBehind())

def waitAbsLidar(angle:int, cm:int, precision=None, decreasing=True):
    '''Waits until lidar at given angle measures smaller (or larger, based on *decreasing*) distance'''
    condition=True
    log.debug("waitabslidar angle %s cm %s decreasing %s"%(angle, cm, decreasing))
    sleep(0.1)
    lastDist=readAbsLidar(angle)
    
    if (decreasing and readAbsLidar(angle)<=cm) or (not decreasing) and readAbsLidar(angle)>=cm:
        beep_parallel()
        log.warning("waitAbsLidar over req. distance! req. dist.: %s, current dist.: %s looking angle %s" % (cm,readAbsLidar(angle),angle))
        return
    while condition:
        distance=readAbsLidar(angle)
        if decreasing:
            condition=distance>=(cm if precision==None else cm+precision)
        else:
            condition=distance<=(cm if precision==None else cm-precision)
        
        if abs(distance-lastDist)>10:
            beep_parallel()
            log.error("waitAbsLidar jump over 10 cm! before: %s , after %s, looking in %s" % (lastDist, distance,angle))
        lastDist=distance
        sleep(0.01)
    if angle!=0: precision=None
    if precision!=None:
        log.debug("distance remaining when switching to waitcm: %s"%readAbsLidar(angle))
        waitCM(readAbsLidar(angle)-cm)
    log.debug("waitAbsLidar over, degree: %s, target cm: %s, actual cm: %s"%(angle,cm,readAbsLidar(angle)))
    return lastDist

def waitCM(cm:int):
    '''Waits until robot has traveled given centimeters.'''
    sleep(0.01)
    p0=getMotorPos()
    c0=packetCount
    t0=time.time()
    t00=time.time()
    log.debug("waitcm p0: %s count0: %s"%(p0,packetCount))
    if cm>0:
        while getMotorPos()<=(p0+cm*TICKS_PER_CM):
            if time.time()-t00>0.02:
                log.warn("waitcm lagged %s"%(time.time()-t00))
            t00=time.time()
            sleep(0.01)
    else:
        while getMotorPos()>=(p0+cm*TICKS_PER_CM): 
            sleep(0.01)
    
    log.debug("waitcm finished, pos: %s count: %s avg count/sec: %s"%(getMotorPos(),packetCount,((packetCount-c0)/(time.time()-t0))))
def getHeading():
    '''Returns last heading value'''
    return (getRawHeading()/10)-heading0

def angleDiff(angle1,angle2)->int:
    '''Returns the shortest distance between two angles'''
    diff=((angle1-angle2)+3600)%360
    if diff>180:
        diff=diff-360
    return diff
WAIT_FOR_HEADING_TOLERANCE:int=2
'''The robot will consider itself at the correct angle if actual angle is only off by this much'''
def waitForHeading(target=None, tolerance=None, turnDir=0):
    '''Waits until robot faces pilotHeadingTarget (variable) angle
    tolerance: Customizable tolerance, default is the constant WAIT_FOR_HEADING_TOLERANCE (2)
    direction: 1: robot is turning right, -1: robot is turning left, 0: any direction
    '''
    if target==None: target=pilotHeadingTarget
    if tolerance==None: tolerance=WAIT_FOR_HEADING_TOLERANCE
    log.debug("waiting for heading %s tolerance: %s turnDir: %s"%(pilotHeadingTarget,tolerance,turnDir))
    if turnDir==0:
        while abs(angleDiff(getHeading(),target))>tolerance:
            sleep(0.01)
    else:
        while angleDiff(getHeading(),target)*turnDir*-1>=tolerance:
            # log.debug("ad %s h %s"%(angleDiff(getHeading(),pilotHeadingTarget),getHeading()))
            sleep(0.01)
    log.debug("wait for heading done at %s"%getHeading())

def stop(breakForce:int=None, wait:bool=True):
    '''Stops the robot
    breakForce: If given, default breaking (counter-driving) force is overridden
    wait: Whether the program should wait until the robot has stopped before resuming'''
    global actSpeed
    global targetSpeed
    targetSpeed=0
    log.debug("stopping , wait? %s"%wait)
    if breakForce!=None:
        if breakForce==0:
            setVMode(0)
        else:
            setBrakeForce(breakForce)
            setVMode(-2)
    else:
        setVMode(-2)
        
    if wait:
        while getVMode()!=VMODE_STOP: sleep(0.01)
        
        sleep(0.1)
    actSpeed=0
    log.debug("stop done (wait? %s)"%wait)


def setHeadingTarget(target:int):
    '''Communicates with the ESP what the target angle is'''
    sendCommand(CMD_SET_TARGET_YAW,int((target+heading0)*10))
targetSpeed:int=0
'''Target speed used for raspberry pi side acceleration and deceleration'''

tempo:int=0
'''Defines the general speed of the robot. Stopping does not set this to 0'''

accelerationForward=5000 #5000
'''Acceleration constant, in tick/second^2'''
accelerationBackward=5000 #5000
actSpeed:int=0
'''Actual speed in tick/second, used for raspberry pi side acceleration and deceleration'''

def setTargetSpeed(tSpeed:int):
    '''Sets the raspberry pi side target speed'''
    global targetSpeed
    targetSpeed=tSpeed
    # setSpeed(tSpeed)

def setTempo(t:int):
    global tempo
    tempo=t
    setTargetSpeed(t)
lidarRevT0=time.time()

@func_thread()
def pilotLoop():
    '''Loop responsible for logging information and wall following'''
    global wIntegral
    global wLastError
    global actSpeed
    global lidarRevT0
    t0=time.time()
    while True:
        if time.time()-t0>0.02:
            log.warn("log thread took too long %s"%(time.time()-t0))
        t0=time.time()
        if LidarService.newLidarDataFlag:
            LidarService.newLidarDataFlag=False
            logLidar()
            if time.time()-lidarRevT0>0.3:
                log.error("Lidar took too long! %s"%(time.time()-lidarRevT0))
                beep_twice_parallel()
            lidarRevT0=time.time()
        if len(log.logs)>0:
            log.save_logs_to_file()
            log.save_temp_logs_to_file()
    
        
        # print(getHeading())
        # print("d: "+str(readAbsLidar(getHeading())))
        # updBehindCM()
        if pilotMode==PILOT_NONE:
            pass
        elif pilotMode==PILOT_FOLLOW_LEFT:
            pilotDistFromWall=readAbsLidar(pilotHeadingTarget-90)
            # print(pilotDistFromWall)
            error=(pilotDistFromWall-wallTarget)
            wIntegral+=error
            correction=-1*(error*wkP+wIntegral*wkI+(error-wLastError)*wkD)
            # display_data(error)
            if correction<-45: correction=-45
            if correction>20: correction=20
            # print(correction)
            setHeadingTarget(pilotHeadingTarget+correction)
            # sendCommand(CMD_SET_TARGET_YAW,int((pilotDir-correction+yaw0)*10))
            wLastError=error
        sleep(0.01)

lidarLogJSON={"Data":[],"T":-1,"DOI":[],"Rect":[]}
'''Dictionary storing logging data'''
lidarDOI=[]
'''Lidar Degrees Of Interest, degrees that were inspected by readLidar, waitAbsLidar or in any other way'''
lidarRects=[]
'''Rectangles that were checked for the nearest point by findNearestPoint'''
pixyObjects=[]
'''Objects detected by the pixy camera'''

labels = []




def logLidar():
    '''Logging function, logs in a file the lidar distance map, gyro, degrees of interest, checked rectangles, pixy camera objects, camera cutoff lines'''
    global lidarDOI
    global lidarRects
    global pixyObjects
    global DISTANCE_MAP
    if pixyObjects==[]: pixyObjects="Null"

    # DISTANCE_MAP[90]=getTOF()
    lidarLogJSON["h0"]=heading0
    lidarLogJSON["heading"]=getHeading()
    lidarLogJSON["Data"]=DISTANCE_MAP #TODO correct LIDAR_ANGLE_OFFSET somehow
    lidarLogJSON["T"]=time.time()
    lidarLogJSON["DOI"]=lidarDOI
    lidarLogJSON["Rect"]=lidarRects
    lidarLogJSON["cutoffLeft"]=CAM_LEFT_CUTOFF
    lidarLogJSON["cutoffRight"]=CAM_RIGHT_CUTOFF
    lidarLogJSON["cutoffTop"]=CAM_TOP_CUTOFF
    lidarLogJSON["cutoffBot"]=CAM_BOT_CUTOFF
    lidarLogJSON["pixyObjects"]=pixyObjects
    lidarLogJSON["labels"] = labels
    lidarRects=[]
    lidarDOI=[]
    pixyObjects=[]
    t0=time.time()
    dumped=json.dumps(lidarLogJSON)
    if time.time()-t0>0.1:
        log.error("JSON DUMP over %s"%(time.time()-t0))
    t0=time.time()
    log.lidar(dumped)
    if time.time()-t0>0.1:
        log.error("Dump over, elapsed %s"%(time.time()-t0))
        beep_parallel()

def add_label(var_name: str):
    ...

def checkAngle(angle:int)->bool:
    '''DEPRECATED Old function used to determine wether there is a wall at given angle'''
    dir=1
    if angle>180: dir=-1
    x=readLidar(90*dir)
    # x=readLidar(90)

    l=readLidar(angle)
    alpha=(90-angle*dir+36000)%360
    correctC=x/cos(alpha/180*pi)
    actualC=readLidar(angle)
    # print(actualC)
    # print(correctC)
    return actualC>correctC+10

def checkSide(side)->int:
    '''Old function used to determine wether there is a wall on a side'''
    corrects=0
    dir=copysign(1,side)
    for i in range(0,80):
        if dir==1:
            if checkAngle(i):
                corrects+=1
        else:
            if checkAngle(360-i):
                # print(360-i)
                corrects+=1
            # print("found at "+str(i))
    return corrects
testVar="TEST"



def openChallengeRun():
    '''The open challenge robot run code'''
    global heading0
    global pilotHeadingTarget
    global direction
    global defaultSpeed
    global direction
    global testVar
    initLoop(open=True)
    middle = True
    defaultSpeed=defaultSpeed
    t00=time.time()
    if readAbsLidar(90) < 15:
        log.info("wall from right")
        direction = 1
        middle = False
    elif readAbsLidar(270) < 15:
        log.info("wall from left")
        direction = -1
        middle = False

    if not middle:
        log.info("targeting middle")
        go(defaultTurnSpeed,-25*direction)
        waitAbsLidar(-90*direction,30)
        go(defaultTurnSpeed,0)
    else:
        go(defaultSpeed,0)
    if middle:
        waitAbsLidar(0,120)
        right=readAbsLidar(90)
        front=readAbsLidar(0)
        left=readAbsLidar(-90)
        rightWall=findNearestPoint((right-10,front-50),(right+10,front-20))
        leftWall=findNearestPoint((-left-10,front-50),(-left+10,front-20))
        if leftWall[0]!=-1:
            direction=DIRECTION_RIGHT
        else:
            direction=DIRECTION_LEFT
    log.info("setting dir to %s"%direction)
    for i in range(12):
        log.debug("i %s"%i)
        if i!=0:
            sleep(0.5)
        waitAbsLidar(0,130)
        setTargetSpeed(defaultTurnSpeed)
        if direction==DIRECTION_RIGHT:
            waitAbsLidar(0,67)
        else:
            waitAbsLidar(0,74)
        arc(90*direction, defaultTurnSpeed)
        heading0+=90*direction
        go(defaultSpeed,0)
    go(defaultTurnSpeed,0)
    waitAbsLidar(0,180)
    t0=time.time()
    while(time.time()-t0<2):
        stop() #ABS
        sleep(0.02)
    camQueue.put("close")
    log.critical("TIME: %s"%(time.time()-t00))
lane:float=0
'''Current lane variable'''
MIDDLE_LANE=0
'''Middle lane'''
LANE_LEFT=-1
'''Left lane'''
LANE_RIGHT=1
'''Right lane'''
LANE_LEFT_OBS=-0.5
'''Lane of traffic signs (left)'''
LANE_RIGHT_OBS=0.5
'''Lane of traffic signs (right)'''

DETECTION=2
'''Turncorner to detect obstacles'''

OUTER_LANE=-1
'''Variable used to improve code readabilty'''

INNER_LANE=1
'''Variable used to improve code readabilty'''

INNER_OBS_LANE=0.5

OUTER_OBS_LANE=-0.5

DIRECTION_RIGHT=1
'''Right direction, the robot turns right at the corners'''
DIRECTION_LEFT=-1
'''Left direction, the robot turns left at the corners'''

parkSide=-1
PARKSIDE_EARLY=0
'''Parking space is at the start of the section'''
PARKSIDE_LATE=1
'''Parking space is at the end of the section'''

SETLANE_MINIMUM=3
ARC_OFFSET_CM_LONG=1
ARC_OFFSET_CM_SHORT=-3 #during a short arc go this much less
SETLANE_SHORT=10
SETLANE_SHORT_OFFSET=-6
SETLANE_REGULAR=25
SETLANE_REGULAR_OFFSET=-10
SETLANE_SHORTEST_OFFSET=-2
'''How many centimeters the robot gets closer to the wall while turning its steering wheel. Approximation.'''

REAR_AXLE_TO_LIDAR_CM=13.5
REAR_AXLE_CENTER_TURNING_RADIUS=39/2
def setLane(wallDirection, targetDistance, is_small=False, turnCorrection=0, already_arc=False):
    '''Internal function, moves the robot to targetDistance from the wall
    wallDirection: -1/1 Which wall to move relative to (inner or outer)
    targetDistance: How close to the wall the robot should go
    is_small: Which preset to use'''
    log.info("setlane dir %s tdist %s"%(wallDirection,targetDistance))
    d0=readAbsLidar(90*wallDirection)
    log.debug("delta d0 %s"%(abs(targetDistance-d0)))
    if is_small:
        angle=30
        correctionY=1
    else:
        angle=40
        correctionY=0
    angle*=wallDirection*copysign(1,d0-targetDistance)
    correctionY*=copysign(1,d0-targetDistance)*-1
    if not already_arc:
        if abs(d0-targetDistance)<SETLANE_MINIMUM:
            go(0)
            return
        arc(angle,blocking=False)
    if d0>targetDistance:
        while readAbsLidar(90*wallDirection)+correctionY>targetDistance:
            if copysign(getHeading(),angle)>=abs(angle):
                setArccancel()
                go(angle)
            sleep(0.01)
    else:
        while readAbsLidar(90*wallDirection)+correctionY<targetDistance:
            if copysign(getHeading(),angle)>=abs(angle):
                setArccancel()
                go(angle)
            sleep(0.01)
    if getSMode()==SMODE_ARC: setArccancel()
    log.debug("over alpha %s x %s"%(getHeading(),readAbsLidar(90*wallDirection)))
    log.debug("setlane over")
    log.debug("angle %s dist1 %s corr %s"%(angle,readAbsLidar(90*wallDirection)-targetDistance,correctionY))
    # go(defaultSpeed,0)
    arc(0+turnCorrection)

targetLeftWallDist=-1
'''Current (target) distance of the robot's (LiDAR) distance from the left wall. Set by switchlane'''
def switchLane(newLane:int, insideWall:bool=False, turnCorrection:int=0, already_arc:bool=False):
    '''Switches the lane to the given new lane
    newLane: One of the constants (LANE_LEFT, LANE_RIGHT, LANE_MIDDLE)
    insideWall: True if the robot is moving between two obstacles'''
    global lane,targetLeftWallDist
    #insidewall actually left wall
    log.info("switching lane to %s from %s ,steep? %s parkpos %s"%(newLane,lane,insideWall,parkPos))
    if newLane==lane:
        log.info("switchlane to same lane")
        go(0)
        pass
    else:
        targetDist=laneWallDists[int(newLane*2+2)]
        targetLeftWallDist=targetDist
        if insideWall:targetDist=100-targetDist
        if direction==DIRECTION_RIGHT:
            setLane(-1 if not insideWall else 1,targetDist,abs(newLane-lane)<=1, turnCorrection, already_arc)
        else:
            setLane(1 if not insideWall else -1,100-targetDist, abs(newLane-lane)<=1, turnCorrection, already_arc)

        lane=newLane
            
        log.info("switchlane done (at: %s from opposite wall)"%(readAbsLidar(0)))
def correctWallDist(wallDirection):
    log.debug("correctWallDist %s actLeftWallDist %s"%(wallDirection,targetLeftWallDist))
    d0=readAbsLidar(90*wallDirection) if wallDirection==DIRECTION_LEFT else (100-readAbsLidar(90*wallDirection))
    f0=readAbsLidar(0)
    log.debug("d0 %s front: %s"%(d0,f0))
    # setSGyroLimits(SGYRO_LIMITED)
    if d0>targetLeftWallDist:
        go(-15)
        while (readAbsLidar(90*wallDirection) if wallDirection==DIRECTION_LEFT else 100-readAbsLidar(90*wallDirection))>=targetLeftWallDist: sleep(0.01) 
    else:
        go(15)
        while (readAbsLidar(90*wallDirection) if wallDirection==DIRECTION_LEFT else 100-readAbsLidar(90*wallDirection))<=targetLeftWallDist: sleep(0.01)
    go(0)
    waitForHeading()
    setSGyroLimits(SGYRO_NORMAL)
    log.debug("correctWallDist finished, target %s actual %s frontdist %s"%(targetLeftWallDist,(readAbsLidar(90*wallDirection) if wallDirection==DIRECTION_RIGHT else 100-readAbsLidar(90*wallDirection)),readAbsLidar(0)))
LATENCY_RIGHT=120
LATENCY_LEFT=120 #218
def arc(toDegree, backwards=False, speed_override=None, percent=100, degTolerance=10, steep=False, blocking=True):
    '''Turns the steering wheel to a percentage then goes until target degree is reached
    toDegree: Target degree
    speed: Speed of the robot, default is the variable defaultSpeed
    percent: How much should the steering wheel turn in percentage. Default is 100'''
    global pilotHeadingTarget
    turnDir=0
    log.debug("arc toDegree %s w speed %s turning percent %s"%(toDegree,speed_override,percent))
    setSteerMode(SMODE_NONE)
    if toDegree>getHeading():
        if not backwards:
            setGyroLatencyMillis(LATENCY_RIGHT)
        else:
            setGyroLatencyMillis(LATENCY_LEFT)
        steer(percent if not backwards else -percent)
        turnDir=1
    else:

        steer(-percent if not backwards else percent)
        if not backwards:
            setGyroLatencyMillis(LATENCY_LEFT)
        else:
            setGyroLatencyMillis(LATENCY_RIGHT)
        turnDir=-1
    if steep:
        sleep(0.2)
    if speed_override!=None: setTargetSpeed(speed)
    else: setTargetSpeed(tempo if not backwards else -tempo)
    
    pilotHeadingTarget=toDegree
    setHeadingTarget(toDegree)
    setArcDir(turnDir)
    log.debug("s %s"%getSMode())
    setSteerMode(SMODE_ARC)
    log.debug("s %s"%getSMode())
    setVMode(-1 if backwards else 1)
    if blocking:
        while getSMode()!=SMODE_ARC: sleep(0.01)
        log.debug("arc blocking smode arc")
        while getSMode()==SMODE_ARC: sleep(0.01)
        log.debug("arc blocking smode _not_ arc")
    # waitForHeading(tolerance=degTolerance,turnDir=turnDir)
    # setSteerMode(SMODE_GYRO)
    # waitForHeading(tolerance=3,turnDir=turnDir)
    log.info("arc over at %s (blocking? %s)"%(getHeading(),blocking))

def toAbsLane(relLane):
    '''Convert LANE_LEFT, LANE_RIGHT to OUTER_LANE, INNER_LANE'''
    return relLane*direction

def turnCorner(target, turnCorrection=0, detection_careful=False, cheat_arc=False):
    '''Turns the robot in the corner, to the specified TURNTARGET. \n
    turnCorrection: End angle correction, "how many extra degrees to turn towards the inner wall" \n
    detection_careful: If true performs a classic turn with backing up. With true, target's only values are the two OBS lanes!'''
    global heading0, targetLeftWallDist, lane
    log.info(f"turncorner target {target}, turnCorrection: {turnCorrection}, back_up?: {detection_careful}")
    if detection_careful:
        waitAbsLidar(0,95)
        setTempo(slowSpeed)
        absLane=toAbsLane(lane)
        if absLane==OUTER_LANE or absLane==OUTER_OBS_LANE:
            target_turn_dist=(40+TURN_RADIUS) if target==TURNTARGET_OUTER_OBS else (60+TURN_RADIUS)
            waitAbsLidar(0, target_turn_dist)
            arc(70*direction)
            stop()
            go(90*direction, backwards=True)
            waitForHeading(90*direction)
        else: #inner, inner_obs
            if absLane==INNER_LANE:
                waitAbsLidar(0,90)
                go(-20*direction)
                waitAbsLidar(0,70)
                go(0)
            target_turn_dist=(40-TURN_RADIUS-4) if target==TURNTARGET_OUTER_OBS else (60-TURN_RADIUS-4)
            waitAbsLidar(0,target_turn_dist)
            stop()
            arc(90*direction, backwards=True)
        waitLidarBehind(38)
        stop()
    else:
        target_turn_dist=turnDists[int(target)]
        waitAbsLidar(0,target_turn_dist+30)
        setTempo(slowSpeed)
        waitEnsureDist(target_turn_dist, 5)
        if not cheat_arc:
            arc((90+turnCorrection)*direction)
        else:
            arc((90+turnCorrection)*direction, blocking=False)
            waitForHeading(90*direction)
    lane=turnTargetToLane(target)
    targetLeftWallDist=laneWallDists[int(lane*2 + 2)]
    log.debug(f"set leftwalldist {targetLeftWallDist}")
    heading0+=90*direction
    log.info("Turncorner over")

def calcLaneFromObs(signColumn, signColor):
    log.debug(f"calcLaneFromObs signColumn {signColumn}, signColor {signColor}")
    if signColor==GREEN:
        return LANE_LEFT if signColumn==-1 else LANE_LEFT_OBS
    elif signColor==RED:
        return LANE_RIGHT if signColumn==1 else LANE_RIGHT_OBS
    else:
        log.warn(f"Illegal obs code: column: {signColumn}, color: {signColor}")
def calcLaneFromStoredObs(sign): return calcLaneFromObs(copysign(1,sign), abs(sign))
def laneToTurnTarget(lane):
    return lane*(-direction)*2 + 2
def turnTargetToLane(turnTarget):
    return (turnTarget-2)/2*(-direction)
lastHeadingLock=threading.Lock()

#This function gets called 20 times a second
def accelerate():
    '''Accelerates'''
    global actSpeed
    # # updHeading()
    
    if actSpeed!=targetSpeed:
        if actSpeed>targetSpeed: actSpeed=(max(actSpeed-accelerationBackward/20,targetSpeed))
        if actSpeed<targetSpeed: actSpeed=(min(actSpeed+accelerationForward/20,targetSpeed))
        log.info("setting speed %s"%actSpeed)
        setSpeed(actSpeed)
    # pass
SerialCommunicationService.accelFunc=accelerate
LidarService.getMotorPosLidar=getMotorPos
RUN: bool = False
'''Used for starting the robot run'''

@func_thread()
def checkForInputLoop():
    '''Function to detect input and act accordingly'''
    global RUN
    global outLoop
    while True:
        inputted = input()
        match inputted:
            case "run":
                RUN = True
            case "exit":
                displayString("EXIT")
                GPIO.cleanup()
                log.critical("__EXIT__")
                os._exit(1)
            case "test":
                if not outLoop:
                    outLoop = True
                else:
                    outLoop = False
                
            case text if text.startswith("exec "):
                try:
                    exec(text[5:])
                except Exception as e: log.error(e)

def checkAngleFromWall()->int:
    '''Calculates the robots angle relative to the wall'''
    a=readLidar(90)
    c=readLidar(60)
    beta=30/180*pi
    #cosine theorem: a^2=b^2+c^2-2*b*c*cos(alpha)
    b=sqrt(a**2+c**2-2*a*c*cos(beta))
    #law of sines: a/sin(alpha)=b/sin(beta)
    alpha=asin(sin(beta)*a/b)
    gamma=pi-alpha-beta
    return gamma*180/pi

def checkLidarDeadzone()->tuple[int,int]:
    '''Checks the lidar deadzone's limits'''
    leftLimit=0
    rightLimit=0
    for i in range(180):
        # log.info("%s %s %s"%(i,DISTANCE_MAP[(i-90+3600)%360],DISTANCE_MAP[(-i-90+3600)%360]))
        if DISTANCE_MAP[(i-90+3600)%360]<20 and rightLimit==0:
            rightLimit=-i
        if DISTANCE_MAP[(-i-90+3600)%360]<20 and leftLimit==0:
            leftLimit=i
    return (leftLimit,rightLimit)



#Startup display mode constants
DISPLAY_LIDAR=0
DISPLAY_LK=1
DISPLAY_ENC0=2
DISPLAY_ENC1=3
DISPLAY_IMU=4
DISPLAY_BH=5
DISPLAY_LIDAR_DZ=6
DISPLAY_ANGLE_FROM_WALL=7

#Diameter of 70
SERVO_MAX=407 #right
SERVO_MIN=229 #left
SERVO_CENT=295 #290 302 300(ferde jobbra)
SERVO_CORRECTED_MAX=SERVO_MAX-(1)#right
SERVO_CORRECTED_MIN=SERVO_MIN+(-9) #left

#ARC RIGHT and ARC LEFT diameter (rear axle center-rear axle center)=36 cm

EMERGENCY_START_PIN=4

def correctHeading(side):

    angle1=30*side
    angle2=120*side
    l1=readLidar(angle1)
    l2=readLidar(angle2)
    alpha=atan(l1/l2)/pi*180

    # alpha=atan((l1-l2*cos(abs(angle2-angle1)/180*pi))/l2*sin(abs(angle2-angle1)/180*pi))/pi*180

    corr=90*side-(angle1-alpha)
    log.debug("corr: %s alpha %s"%(corr,alpha))
    return corr
cam:Picamera2
def initLoop(open=False):
    '''Displays debug information and waits until a button is pressed
    open: Set to true to display open challenge relevant information'''
    global heading0
    global direction
    global outLoop
    global parkPos
    global parkSide
    global cam, defaultSpeed, slowSpeed, superSlowSpeed, defaultTurnSpeed, camP
    if open:
        defaultSpeed=3000
        defaultTurnSpeed=2000
    else: 
        defaultTurnSpeed=1000
        defaultSpeed=1300
        slowSpeed=800
        superSlowSpeed=500
    beep()
    dispMode=DISPLAY_LIDAR
    displayData(10101010)
    log.info("10101010")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EMERGENCY_START_PIN,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    checkForInputLoop()
    outLoop = False
    
    
    SerialCommunicationService.startReading()
    setGyroError(-4) #-14 or -4
    setGyroLatencyMillis(240) #always overwritten!
    getRawHeading()
    setServoMin(SERVO_CORRECTED_MIN) #SERVO SAFE LIMIT
    setServoMax(SERVO_CORRECTED_MAX) #SERVO SAFE LIMIT
    setServoCent(SERVO_CENT)

    setHandbrakeP(00) #200

    if open:
        setGyroPD(2,30)
    else:
        setGyroPD(5,40)
    setSpeedcontrolPID(7,0,0)
    setSGyroLimits(SGYRO_NORMAL)
    log.info("SET")
    sleep(0.2)
    rh=getRawHeading()
    log.info("rawheading %s"%rh)
    heading0=rh/10
    
    setBrakeForce(50)
    pilotLoop()
    
    last_qyro = getHeading()
    pressed:bool=False
    while not pressed:
        sleep(0.5)
        parkPos=-1
        heading0=getRawHeading()/10
        if TM.switches[0] or RUN or (GPIO.input(EMERGENCY_START_PIN)==GPIO.LOW):
            setLeds("11111111")
            sleep(1)
            pressed=True
        setLed(0, last_qyro != heading0)
        last_qyro = heading0
        objPos:tuple=(-2,-2)
        color:int=None
        if dispMode==DISPLAY_LIDAR: #display lidar distance at angle 0
            displayData("L.",(readLidar(0)+readLidarBehind()+2.5))
        elif dispMode==DISPLAY_ENC0: #display left motor encoder
            displayData("E0.",getLPos())
        elif dispMode==DISPLAY_ENC1: #display right motor encoder
            displayData("E1.",getRPos())
        elif dispMode==DISPLAY_IMU: #display gyroscope
            displayData("I.",getRawHeading())
        elif dispMode==DISPLAY_LK: #turns on every led to check led&key
            displayString("88888888")
            setLeds("11111111")
        elif dispMode==DISPLAY_BH: #displays laser distance sensor
            displayData("B.",readLidarBehind())
            # displayData("B.",-1)
        elif dispMode==DISPLAY_LIDAR_DZ: #displays lidar deadzone limits
            limits=checkLidarDeadzone()
            displayString("D.%s  %s"%(limits[0],limits[1]))
        elif dispMode==DISPLAY_ANGLE_FROM_WALL:
            displayData("A.",checkAngleFromWall())
        if TM.switches[1]: #switch display mode
            displayString("--------")
            sleep(0.2)
            dispMode=(dispMode+1)%8
        if TM.switches[7]:
            displayString("EXIT")
            log.critical("__EXIT__")
            GPIO.cleanup()
            closeCam()
            os._exit(1)
        left=readAbsLidar(-90)
        right=readAbsLidar(90)
        if right<20 and left>20:
            direction=DIRECTION_LEFT
            setLed(6,True)
            setLed(7,False)
        else:
            direction=DIRECTION_RIGHT
            setLed(7,True)
            setLed(6,False)
    log.info("Initloop over")
    return color, objPos

trafficSignMatrix = [
    [0,0,0],
    [0,0,0],
    [0,0,0],
    [0,0,0]
]
'''For storing the detected traffic signs'''

section = 0
'''Current section'''

def findFirst(section):
    '''Returns the color (and column, signed) of the first traffic sign in the section'''
    colorPos=0
    log.info("ff section: %s"%trafficSignMatrix[section%4])
    for i in range(3):
        if trafficSignMatrix[section%4][i]!=0:
            log.info("findfirst %s i %s"%(trafficSignMatrix[section%4][i] ,i))
            return trafficSignMatrix[section%4][i],i

def findLast(section):
    '''Returns the color and position (from last) of the last traffic sign in the section'''
    colorPos=0
    for i in range(3):
        if trafficSignMatrix[section%4][2-i]!=0: return trafficSignMatrix[section%4][2-i],i

def findLastInLap():
    log.info("find last in lap, matrix: %s"%trafficSignMatrix)
    if trafficSignMatrix[0][0]!=0 or trafficSignMatrix[0][1]!=0:
        return findFirst(0)
    else:
        return findLast(3)
dontReverse = False
'''Used for optimization'''
def turnAround():
    '''180° turn around a red traffic sign'''
    # global dontReverse

    global heading0
    global doDetection
    global lane
    doDetection=False
    if lane==LANE_RIGHT: #always the case
        arc(-90,speed=safeSpeed)
        idealDistance=40+leftLaneOffset
        if readAbsLidar(-90)>idealDistance:
            waitAbsLidar(-90,idealDistance)
        else:
            stop()
            go(-90, backwards=True)
            waitAbsLidar(-90,idealDistance+5,decreasing=False)
            stop()
            go(-90)
            waitAbsLidar(-90,idealDistance)
        arc(-180,speed=safeSpeed)
        heading0-=180

parkPos=0
'''The position of the parking space (always 0, meaning the starting section)'''
doDetection=True
'''Set to false if all traffic signs are detected'''
defaultSpeed=3200
'''The default speed, most functions use this by default'''
defaultTurnSpeed=1500
safeSpeed=2000
'''Slower speed'''
def reverseMatrix():
    '''Reverses the trafficSignMatrix when turning around. Also sets the parking space position accordingly'''
    global trafficSignMatrix
    global parkPos, parkSide
    parkPos=(4-parkPos)%4
    parkSide=1-parkSide
    trafficSignMatrix[1],trafficSignMatrix[3]=trafficSignMatrix[3],trafficSignMatrix[1]
    for i in range(len(trafficSignMatrix)):
        trafficSignMatrix[i].reverse()
        for j in range(len(trafficSignMatrix[i])):
            trafficSignMatrix[i][j]*=-1
rightLaneOffset, leftLaneOffset=0,0
'''Lane offsets for when there is a parking space next to the robot'''
startTime=-1
'''T0, when the robot starts'''
finalSection=12
'''Final section variable, if the robot turns around is decreased by 1'''

def investigateObjectColor(ObjPos:Object,back=True,useLED=True, goBack=50)->int:
    global lane
    # signColor=GREEN
    signColor=detectObjectColor(ObjPos)
    log.debug("COC signcolor=%s"%signColor)
    log.debug("signObj %s %s"%(ObjPos.center[0],ObjPos.center[1]))
    # ox,oy=ObjPos.center
    # log.debug("COC ox,oy=%s %s"%(ox,oy))
    # # ox-=getAbsX()
    # # oy-=getAbsY(back=back)
    # oy+=6
    # distance=sqrt(ox**2+oy**2)
    # angle=atan(ox/oy)*180/pi
    # log.info("obj found %s dist %s angle %s rel x %s y %s"%(ObjPos,distance,angle,ox,oy))
    # go(defaultSpeed,angle)
    # # pixy.set_lamp(1,1)
    # waitCM(distance-30)
    # stop()
    # lane=0
    # # signColor = checkColor()
    
    # # pixy.set_lamp(0,0)
    # go(-defaultSpeed,0)

    # waitCM(-goBack)
    # stop()
    return signColor
def isInParkingSection()->bool:
    return section%4==parkPos%4 and parkPos!=-1
slowSpeed=500
superSlowSpeed=350

def setOffsets():
    global rightLaneOffset, leftLaneOffset
    if isInParkingSection():
        if direction==DIRECTION_LEFT:
            rightLaneOffset=20
            leftLaneOffset=0
        elif direction==DIRECTION_RIGHT:
            rightLaneOffset=0
            leftLaneOffset=20
    else:
        rightLaneOffset=0
        leftLaneOffset=0
    log.debug("setoffsets section %s l %s r %s"%(section, leftLaneOffset, rightLaneOffset))
def getFirstTurnTarget(actSection):
    actSection%=4
    f_obs, i=findFirst(actSection)
    f_lane=calcLaneFromStoredObs(f_obs)
    l_obs=trafficSignMatrix[actSection%4][2]
    if i==0 and (abs(l_obs)==abs(f_obs)): #two obstacles of same color
        log.debug(f"f_obs: {f_obs} l_obs: {l_obs}")
        if abs(calcLaneFromStoredObs(l_obs))>abs(f_lane): #last obs is "more outer" from middle_lane
            f_lane=calcLaneFromStoredObs(l_obs)
    return laneToTurnTarget(f_lane)
def getLastObjLane(actSection):
    '''Returns the last (and possibly only) lane we need to switch to in actSection'''
    return calcLaneFromStoredObs(findLast(actSection)[0])
def isChangelane(actSection):
    '''Returns true if the robot needs to change lanes in the middle of actSection'''
    return abs(findFirst(actSection%4)[0])!=abs(findLast(actSection%4)[0])

def waitEnsureDist(target_dist, allowed_deviation=0):
    '''Ensures robot is target_dist cm away from front wall. Only straight and forward\nallowed_deviation: How many cm to allow (without backing up)'''
    actual_dist=readAbsLidar(0)
    log.info(f"EnsureDist: target: {target_dist}, allowed deviation: {allowed_deviation}, actual dist: {actual_dist}")
    if actual_dist<target_dist-allowed_deviation: #need to backup
        log.warn(f"EnsureDist: too close to wall! target: {target_dist}, actual: {actual_dist}")
        stop()
        go(0, backwards=True)
        waitAbsLidar(0,target_dist,decreasing=False)
        stop()
        go(0)
    waitAbsLidar(0,target_dist)
    log.info("EndureDist over")

REAR_LEFT_X=15
REAR_LEFT_Y=8

def parkingManouver(firstSignColor):
    global direction, heading0
    if firstSignColor==(GREEN if direction==DIRECTION_LEFT else RED): #has to avoid first one
        turnCorner(target= TURNTARGET_INNER) #avoid first one, to inner lane
        waitLidarBehind(105,decreasing=False)
        switchLane(LANE_RIGHT_OBS if direction==DIRECTION_LEFT else LANE_LEFT_OBS,insideWall=True) #outer obs
        waitCM(10)
        stop()
        go(0, backwards=True)
        if direction==DIRECTION_RIGHT: waitLidarBehind(100)
        else: waitAbsLidar(0,145,decreasing=False)
        stop()
    else: #does not have to avoid or there isnt one
        turnCorner(target=TURNTARGET_OUTER_OBS) #outer obs
        if direction==DIRECTION_RIGHT: 
            if readLidarBehind()>90:
                stop()
                go(0, backwards=True)
                waitLidarBehind(80,decreasing=True)
                go(0)
            waitLidarBehind(90,decreasing=False)
        else:
            waitAbsLidar(0,165)
    setTempo(superSlowSpeed)
    go(0)

    if direction==DIRECTION_RIGHT: waitLidarBehind(116,decreasing=False)
    else: waitAbsLidar(0,133.5)
    
    arc(90*direction, steep=True)
    if direction==DIRECTION_LEFT: heading0-=180 #the robot parkink is 0°

    #UNIFIED PARKING
    direction=DIRECTION_RIGHT
    waitAbsLidar(90,20) #forward
    stop()
    go(90, backwards=True)
    waitLidarBehind(45) #backing up
    arc(0, backwards=True, blocking=False)
    tooClose=False
    while(getSMode()!=SMODE_ARC): sleep(0.01)
    while (getSMode()==SMODE_ARC and not tooClose):
        if getHeading()<20:
            alpha=getHeading()
            l=readLidarBehind() 
            wallDist=cos(alpha/(360)*2*pi)*l #straight distance from the wall
            beta=atan(REAR_LEFT_Y/REAR_LEFT_X)
            l2=wallDist/cos((alpha+beta)/360*(2*pi))
            c=sqrt(REAR_LEFT_X**2+REAR_LEFT_Y**2)
            dist=l2-c
            if dist<2:
                tooClose=True
                log.warn("Emergency stopping park arc! dist %s l %s wallDist %s beta %s l2 %s c %s"%(dist,l,wallDist,beta,l2,c))
        sleep(0.01)
    if tooClose: #emergency stopped
        setArccancel()
        beep_twice_parallel()
        log.warn("Emergency stopped park arc!")
    log.debug("park arc done")
    stop()
    go(0)
    log.debug("parking go")
    waitAbsLidar(0, 15)
    stop()
    log.debug("park done")

def findFirstObsInNextSection():
    waitAbsLidar(0,100)
    if direction==DIRECTION_RIGHT:
        signObj=findNearestObjectAbs((80, 220), (175, 270), back=False, overrideAbsX=targetLeftWallDist)
        if signObj.empty: signObj=findNearestObjectAbs((175, 220), (220, 270), back=False, overrideAbsX=targetLeftWallDist)
        sx, sy=signObj.toAbsolute().center
        log.debug(f"first obstacle sx: {sx} sy: {sy}")
        signColumn=-1 if sy>250 else 1
        signRow=0 if sx<125 else (1 if sx<175 else 2)
    else:
        signObj=findNearestObjectAbs((-75, 220), (20, 270), back=False, overrideAbsX=targetLeftWallDist)
        if signObj.empty: signObj=findNearestObjectAbs((-120, 220), (-75, 270), back=False, overrideAbsX=targetLeftWallDist)
        sx, sy=signObj.toAbsolute().center
        signColumn=-1 if sy<250 else 1
        signRow=0 if sx>-25 else 1 if sx>-75 else 2
    return signColumn,signRow

def obstacleChallengeRun():
    global direction, startTime, parkPos, parkSide, rightLaneOffset, leftLaneOffset, lane, trafficSignMatrix, section, defaultSpeed, defaultTurnSpeed, targetLeftWallDist
    log.info("Obstacle challenge run started")
    initLoop(open=False)

    # #TEMPORARY TEST
    # lane=LANE_RIGHT
    # direction=DIRECTION_RIGHT
    # setTempo(slowSpeed)
    # defaultSpeed=defaultSpeed
    # defaultTurnSpeed=slowSpeed
    # targetLeftWallDist=80
    # trafficSignMatrix= [
    #     [0,0,RED],
    #     [-GREEN,0,RED],
    #     [RED, 0, -GREEN],
    #     [-RED, 0, -GREEN]
    # ]
    # #TEMPORARY TEST

    log.info("Obs initloop over")
    parkPos=0 #Always in starting section
    parkSide=PARKSIDE_EARLY if direction==DIRECTION_RIGHT else PARKSIDE_LATE
    startTime=time.time()
    log.info("Initloop over, direction %s"%direction)
    section=0
    log.section=section
    optimizedFrom=5
    ##PARK OUT
    setTempo(slowSpeed)
    if direction==DIRECTION_LEFT:
        arc(-65,steep=True)
        stop()
        signObject=findNearestObject((-30,5),(-5,20))
        if not signObject.empty:
            signColor=detectObjectColor(signObject)
            trafficSignMatrix[0][2]=-signColor #always on the left
        if signObject.empty or signColor==RED:
            arc(0,steep=True)
            lane=LANE_RIGHT_OBS
            targetLeftWallDist=60
            go(0)
        else:
            arc(-90,steep=True)
            waitAbsLidar(-90,29.5)
            arc(0,steep=True)
            lane=LANE_LEFT
            targetLeftWallDist=20

    elif direction==DIRECTION_RIGHT:
        arc(35,steep=True)
        stop()
        signObj:Object=findNearestObject((10,5),(40,80))
        if not signObj.empty:
            signColor=detectObjectColor(signObj)
            row=2 if signObj.center[1]>50 else 1
            trafficSignMatrix[0][row]=signColor #always on the right
        else:
            signColor=0
        if signColor==GREEN:
            go(42)
            waitCM(16)
            arc(0)
            lane=LANE_LEFT_OBS
            targetLeftWallDist=40
            go(defaultSpeed,0)
        elif signObj.empty or signColor==RED:
            arc(90,steep=True)
            go(90)
            waitAbsLidar(90,29.5)
            arc(0)
            lane=LANE_RIGHT
            targetLeftWallDist=80
            go(0)
    ##DETECTION
    for i in range(1,4): #repeat, changed 4->12
        go(0)
        section=i
        log.section = section
        waitAbsLidar(0,120)
        setTempo(slowSpeed)
        signColumn, signRow=findFirstObsInNextSection()
        if signRow==0: #obstacle in first lane
            turnCorner(target=TURNTARGET_INNER_OBS if (signColumn*direction==1) else TURNTARGET_OUTER_OBS, detection_careful=True)
            signObj=findNearestObjectAbs((20,80),(80,120))
            sx,sy=signObj.toAbsolute().center
            signColumn=-1 if sx<50 else 1
            #do we overwrite signRow??
            signColor=detectObjectColor(signObj)
            trafficSignMatrix[section%4][signRow]=signColumn*signColor #store
            switchLane(calcLaneFromObs(signColumn, signColor))
            waitLidarBehind(108,decreasing=False)
            signObj = findNearestObjectAbs((20,180),(80,220),back=True, outer=(lane*direction<0)) #first check detection
            if not signObj.empty: #there is an obstacle in row 3
                stop()
                sx, sy = signObj.toAbsolute().center
                log.debug(f"sx {sx} sy {sy}")
                signRow = 2
                signColumn = -1 if sx < 50 else 1
                signColor=detectObjectColor(signObj)
                log.debug(f"found obstacle: row {signRow}, column {signColumn}")
                trafficSignMatrix[section%4][signRow] = signColor * signColumn
                targetLane=calcLaneFromObs(signColumn, signColor)
                if copysign(1,lane)==copysign(1,targetLane) and abs(lane)>=abs(targetLane): #if already in correct lane dont move closer to the middle
                    setTempo(defaultSpeed)
                    waitLidarBehind(130)
                    correctWallDist(copysign(1,lane))
                    pass
                else:
                    switchLane(targetLane, insideWall=isInParkingSection())
                #DONE with second switchlane or correctWallDist
            else:
                setTempo(defaultSpeed)
                waitLidarBehind(130)
                correctWallDist(copysign(1,lane))
                pass
        else:
            turnCorner(target=TURNTARGET_INNER_OBS if (signColumn*direction==1) else TURNTARGET_OUTER_OBS, detection_careful=False) #optimized turncorner
            #take picture from here
            if signRow==1: waitLidarBehind(105,decreasing=False) #TODO fine-tune
            else: waitLidarBehind(125,decreasing=False)
            signObj=findNearestObjectAbs((20,130), (80, 225))
            sx,sy=signObj.toAbsolute().center
            signColumn=-1 if sx<50 else 1
            signColor=detectObjectColor(signObj)
            trafficSignMatrix[section%4][signRow]=signColumn*signColor #store
            newLane=calcLaneFromObs(signColumn, signColor)
            switchLane(newLane, insideWall=True if newLane*direction>0 else False)
    ##EARLY DETECT
        
    go(0)
    section+=1
    log.section = section
    
    signColumn, signRow=findFirstObsInNextSection()
    if signRow==0: #obstacle in first lane
        turnCorner(target=TURNTARGET_INNER_OBS if (signColumn*direction==1) else TURNTARGET_OUTER_OBS, detection_careful=True)
        signObj=findNearestObjectAbs((30,80),(70,120))
        sx,sy=signObj.toAbsolute().center
        signColumn=-1 if sx<50 else 1
        signColor=detectObjectColor(signObj)
        trafficSignMatrix[section%4][signRow]=signColumn*signColor #store
        switchLane(turnTargetToLane(getFirstTurnTarget(section)))
        if isChangelane(section): #first and last traffic sign of section are of different colors
            waitLidarBehind(108,decreasing=False)
            switchLane(getLastObjLane(section), insideWall=True)
        else:
            setTempo(defaultSpeed)
            waitLidarBehind(110, decreasing=False)
            correctWallDist(direction) #Inner wall
    elif signRow==1 and direction==DIRECTION_LEFT:
        turnCorner(target=TURNTARGET_INNER_OBS if (signColumn*direction==1) else TURNTARGET_OUTER_OBS, detection_careful=False) #optimized turncorner
        #take picture from here
        waitLidarBehind(110,decreasing=False)
        signObj=findNearestObjectAbs((30,130), (70, 225))
        sx,sy=signObj.toAbsolute().center
        signColumn=-1 if sx<50 else 1
        signColor=detectObjectColor(signObj)
        trafficSignMatrix[section%4][signRow]=signColumn*signColor #store
        switchLane(calcLaneFromObs(signColumn, signColor))
    else:
        optimizedFrom=4
    ##OPTIMIZED
    for i in range(optimizedFrom,12):
        section=i
        log.section = section
        cheat=False
        if toAbsLane(lane)==INNER_LANE and isChangelane(section):
            log.debug("CHEAT")
            target=getFirstTurnTarget(section)
            if target==TURNTARGET_OUTER or target==TURNTARGET_OUTER_OBS:
                turnCorr=40
                cheat=True
            else:
                turnCorr=-20
        #TEMPORARY TURNOFF
        turnCorr=0
        cheat=False
        #TEMPORARY TURNOFF
        turnCorner(target=getFirstTurnTarget(section), turnCorrection=turnCorr, cheat_arc=cheat)
        if isChangelane(section): #first and last traffic sign of section are of different colors
            if not cheat: waitLidarBehind(108,decreasing=False)
            switchLane(getLastObjLane(section), insideWall=isInParkingSection(), already_arc=cheat)
        else:
            setTempo(defaultSpeed)
            waitLidarBehind(110, decreasing=False)
            correctWallDist(copysign(1,lane) if not isInParkingSection() else direction)
    ##PARKING
    section+=1
    log.section=section
    parkingManouver(abs(trafficSignMatrix[0][0])) #Avoid obstacle if needed then park
        
            

ARC_OFFSET_CM_20=6
def befPark():
    
    park()
def park():
    pass
    #PRECISION PARKING
    # while readAbsLidar(-90*direction)>10.5:
    #     setSteerMode(SMODE_NONE)
    #     steer(-100)
    #     sleep(0.3)
    #     setTargetSpeed(-superSlowSpeed)
    #     setVMode(VMODE_BACKWARD)
    #     waitCM(-1.5)
    #     stop()
    #     arc(0 if direction==DIRECTION_RIGHT else 180, -superSlowSpeed,steep=True)
    #     stop()
    #     # steer(100)
    #     # sleep(0.3)
    #     # setTargetSpeed(-superSlowSpeed)
    #     # setVMode(VMODE_BACKWARD)
    #     # waitCM(-3)
    #     # stop()
    #     setSteerMode(SMODE_NONE)
    #     steer(0)
    #     sleep(0.3)
    #     go(superSlowSpeed,0 if direction==DIRECTION_RIGHT else 180)
    #     waitAbsLidar(0 if direction==DIRECTION_RIGHT else 180,12.5)
    #     stop()

def testAdvancedStrategyInner():
    global direction
    initLoop(open=False)
    direction=DIRECTION_LEFT
    switchLane(LANE_LEFT,insideWall=False)
    setTargetSpeed(slowSpeed)
    waitAbsLidar(0,110)
    signObj=findNearestObjectAbs((-60,220),(10,280), back=False)
    sx, sy = signObj.toAbsolute().center
    signColumn = -1 if sy < 250 else 1
    signRow = 0 if sx > -20 else 1
    if signObj.empty:
        turnCorner(INNER_LANE)
        waitLidarBehind(100,decreasing=False)
        #letárolni!
        stop()
        signObj=findNearestObjectAbs((30,180),(70,220),back=False)
        sx, sy = signObj.toAbsolute().center
        col=detectObjectColor(signObj)
        switchLane(LANE_LEFT if col==GREEN else LANE_RIGHT, insideWall=False)
    else: #there is something

        if signRow==0:
            col=detectObjectColor(signObj)
            go(defaultSpeed,0)
            turnCorner(target=INNER_LANE if (col==GREEN and direction==DIRECTION_LEFT) else OUTER_LANE, turnCorrection=10 if (lane==LANE_LEFT) else -10)

            #last row detection
            waitLidarBehind(100,decreasing=False)
            #letárolni!
            stop()
            signObj=findNearestObjectAbs((30,180),(70,220),back=False)
            #lekezelni ha nincs
            sx, sy = signObj.toAbsolute().center
            col=detectObjectColor(signObj)
            switchLane(LANE_LEFT if col==GREEN else LANE_RIGHT, insideWall=True)
        elif signRow==1:
            defaultSpeed=slowSpeed
            #slow turn!!
            turnCorner(INNER_OBS_LANE if signColumn==1 else OUTER_OBS_LANE)
            signObj=findNearestObjectAbs((20,130),(80,170),back=True)
            col=detectObjectColor(signObj)
            switchLane(LANE_LEFT if col==GREEN else LANE_RIGHT, insideWall=True)

            stop()
            pass

    
    stop()
def testAdvancedStrategyOuter():
    global direction, defaultSpeed
    initLoop(open=False)
    direction=DIRECTION_LEFT
    switchLane(LANE_RIGHT,insideWall=False)
    waitAbsLidar(0,100)
    signObj=findNearestObjectAbs((-60,220),(10,280),back=False)
    sx, sy = signObj.toAbsolute().center
    signColumn = -1 if sy < 250 else 1
    signRow = 0 if sx > -20 else 1
    if(signObj.empty):
        turnCorner(INNER_LANE) #maybe need turncorrection
        waitAbsLidar(0,100) #not stopping, could be out-of-frame
        signObj=findNearestObjectAbs((30,180),(70,220),back=False)
        if(signObj.empty):
            stop()
        else:
            #letárolni!^^
            sx, sy = signObj.toAbsolute().center
            col=detectObjectColor(signObj)
            switchLane(LANE_LEFT if col==GREEN else LANE_RIGHT, insideWall=True)
    else:
        log.debug(f"found in {signColumn} {signRow}, target: {INNER_OBS_LANE if signColumn==-1 else OUTER_OBS_LANE}")
        defaultSpeed=slowSpeed
        turnCorner(target=INNER_OBS_LANE if signColumn==-1 else OUTER_OBS_LANE)
        if signRow==1:
            waitLidarBehind(80,decreasing=False)
        signObj=findNearestObjectAbs((20,90),(80,170),back=True)
        col=detectObjectColor(signObj)
        target_lane=LANE_LEFT if col==GREEN else LANE_RIGHT
        switchLane(target_lane, insideWall=False, turnCorrection=-15*target_lane)
        if signRow==0:
            signObj=findNearestObjectAbs((30,180),(70,220),back=False)
            if(not signObj.empty):
                #problem...
                col=detectObjectColor(signObj)
                if(col==GREEN and lane==LANE_LEFT or col==RED and lane==LANE_RIGHT):
                    correctWallDist(-direction)
                else:
                    switchLane(LANE_LEFT if col==GREEN else LANE_RIGHT,insideWall=True)
                # stop()
                # go(-defaultSpeed,0)
                # waitAbsLidar(0,192,decreasing=False)
                # stop()
                # go(defaultSpeed,0)
                # waitLidarBehind(108,decreasing=False)
                # signObj=findNearestObjectAbs((30,180),(70,220),back=False)
                # col=detectObjectColor(signObj)
                # switchLane(LANE_LEFT if col==GREEN else LANE_RIGHT, insideWall=True)
            else:
                correctWallDist(-direction)


    stop()
    pass
def testRun():
    '''Test run used for testing'''
    global pilotHeadingTarget
    global trafficSignMatrix
    global heading0
    global parkPos, parkSide, section, lane, accelerationBackward
    global leftLaneOffset, rightLaneOffset,defaultSpeed,direction,targetLeftWallDist
    initLoop(open=False)
    direction=DIRECTION_RIGHT
    lane=LANE_RIGHT_OBS
    setTempo(slowSpeed)
    go(0)
    waitAbsLidar(0,150)
    switchLane(LANE_LEFT)
    waitAbsLidar(0,50)
    stop()
    # waitCM(100)
    # stop()
    # go(-superSlowSpeed,0)
    # waitCM(-100)
    # stop()
    # go(superSlowSpeed,0)
    # waitCM(100)
    # stop()
    # waitAbsLidar(0 if direction==DIRECTION_RIGHT else 180,12.5)
    # stop()
    # while readAbsLidar(-90*direction)>10.5:
    #     setSteerMode(SMODE_NONE)
    #     steer(-100)
    #     sleep(0.3)
    #     setTargetSpeed(-superSlowSpeed)
    #     setVMode(VMODE_BACKWARD)
    #     waitCM(-1.5)
    #     stop()
    #     arc(0, -superSlowSpeed,steep=True)
    #     stop()
        # steer(100)
        # sleep(0.3)
        # setTargetSpeed(-superSlowSpeed)
        # setVMode(VMODE_BACKWARD)
        # waitCM(-3)
        # stop()
        # setSteerMode(SMODE_NONE)
        # steer(0)
        # sleep(0.3)
        # go(superSlowSpeed,0)
        # waitAbsLidar(0 if direction==DIRECTION_RIGHT else 180,12.5)
        # stop()
    
    

    sleep(10000)
    
    # direction=DIRECTION_RIGHT
    # lane=LANE_RIGHT
    # if direction==DIRECTION_LEFT:
    #     if lane==LANE_RIGHT:
    #         waitAbsLidar(0,170)
    #         setTargetSpeed(superSlowSpeed)
    #         waitAbsLidar(0,127)

    #         arc(-90,superSlowSpeed)
    #         stop()
            
    #     elif lane==LANE_LEFT:
    #         waitAbsLidar(0,125)
    #         setTargetSpeed(superSlowSpeed)
    #         waitAbsLidar(0,90) #CALIB
    #         stop()
    #         arc(-100,-superSlowSpeed)
    #     go(-superSlowSpeed,-100)
    #     waitAbsLidar(-90,50,decreasing=False)
    #     arc(0,-superSlowSpeed)
    #     stop()
    # elif direction==DIRECTION_RIGHT:
    #     if lane==LANE_RIGHT:
    #         waitAbsLidar(0,70)
    #         setTargetSpeed(superSlowSpeed)
    #         waitAbsLidar(0,140) #CALIB
    #         stop()
    #         arc(-90,-superSlowSpeed)
            
    #     elif lane==LANE_LEFT:
    #         waitLidarBehind(60)
    #         setTargetSpeed(superSlowSpeed)
    #         waitLidarBehind(113)

    #         arc(90,superSlowSpeed)
    #         stop()
            
    #     go(-superSlowSpeed,-90)
    #     waitAbsLidar(90,55,decreasing=False)
    #     arc(0,-superSlowSpeed)
    #     stop()
    # go(superSlowSpeed,0)
    # waitAbsLidar(0,15)
    # stop()
def secondday():
    initLoop(open=False)
    go(defaultSpeed,0)
    turnCorner(OUTER_LANE)
    go(defaultSpeed,0)
    turnCorner(DETECTION)
    stop()
    utas=findNearestObjectAbs((20,80),(80,220))
    if utas.center[1]<180:
        log.debug("<180")
        utas_sz=detectObjectColor(utas)
    else:
        log.debug(">=180")
        go(defaultSpeed,0)
        waitCM(50)
        stop()
        utas=findNearestObjectAbs((20,180),(80,220))
        utas_sz=detectObjectColor(utas)
    if utas_sz==RED:
        log.debug("RED")
        utas=utas.toAbsolute()
        log.debug(utas.center[0])
        if utas.center[0]<50:
            log.debug("LEFT")
            go(defaultSpeed,0)
            setLane(OUTER_LANE,47)
            go(defaultSpeed,0)
            waitAbsLidar(0,50)
            stop()
        else:
            log.debug("RIGHT")
            go(defaultSpeed,0)
            setLane(OUTER_LANE,67)
            go(defaultSpeed,0)
            waitAbsLidar(0,50)
            stop()

match run_type:
    case "2nd day":
        secondday()
    case "Obstacle":
        obstacleChallengeRun()
    case "Open":
        openChallengeRun()
    case "Test":
        testRun()
    case _:
        log.error(f"run_type arguement not assigned -- {run_type}")
        log.save_logs_to_file()
        log.save_temp_logs_to_file()


stop()
#END
camQueue.put("close")
sleep(1)
log.critical("__EXIT__program end")
os._exit(1)