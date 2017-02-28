from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import imutils
import time
import math
from networktables import NetworkTables
import datetime
from datetime import timedelta
import time
import sys

class Camera:
  def __init__(self):
    self.camera = PiCamera()
    self.camera.resolution = (640,480)
    self.camera.framerate = 60
    self.camera.shutter_speed = 500 #works better at 100 but lets in other light, lower = good for filtering
    self.camera.awb_mode = "off"
    self.camera.awb_gains = (1.9, 1.9)
    self.rawCapture = PiRGBArray(self.camera, size = self.camera.resolution)
    self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
    self.frame = None  #does nothing in constructor
    self.stopped = False  #does nothing in constructor
    self.camera.brightness = 50 #50 is default, this does nothing
    self.camera.hflip = True #use to change orientation of camera
    #self.camera.rotation = 180 #Use to change orientation of camera
    self.camera.exposure_mode = "verylong" #other modes work, just not auto
    self.timeStamp = None
  def start(self):
    t = Thread(target = self.update, args = ())
    t.daemon = True
    t.start()
    return self
  def update(self):
    for f in self.stream:
      self.frame = f.array
      self.rawCapture.truncate(0)
      self.timeStamp = time.perf_counter()
      if self.stopped:
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()
        return
  def read(self):
    return self.frame
  def stop(self):
    self.stopped = True
  def getTimeStamp(self):
    return self.timeStamp
    
def getHSVImage(RGBImage, hue, sat, val):
    HSVImage = cv2.cvtColor(RGBImage, cv2.COLOR_BGR2HSV)
    return cv2.inRange(HSVImage, (hue[0], sat[0], val[0]), (hue[1], sat[1], val[1]))
    
def findContours(HSVImage):
    im1, contours, hierarchy = cv2.findContours(HSVImage, mode = cv2.RETR_EXTERNAL, method = cv2.CHAIN_APPROX_SIMPLE)
    return contours
    
def getContourImage(HSVImage, contours):
    return cv2.drawContours(HSVImage, contours, -1, (255, 255, 0), 4)
    
def findBoundingRects(contours, minWidth, maxWidth, minHeight, maxHeight):
    filteredRects = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if w >= minWidth and w <= maxWidth and h >= minHeight and h <= maxHeight:
            filteredRects.append((x,y,w,h))
    return (filteredRects)

def getXMiddleOfRect(rect):
  x = rect[0]
  width = rect[2]
  return x + width / 2

def filterTargetRects(rects):
  xTolerance = 5
  counter = 0
  while counter < len(rects):
    rectA = rects[counter]
    i = counter+1
    while i < len(rects):
      rectB = rects[i]
      xDiff = math.fabs(getXMiddleOfRect(rectB) - getXMiddleOfRect(rectA))
      if xDiff <= xTolerance:
        return rectA
      i = i + 1
    counter = counter + 1
    
def distanceToTarget(targetY):
    return targetY #todo: need to implement

def angleToTarget(targetX, targetWidth): #targetX is leftmost point    400 250
  midpoint = targetX + targetWidth / 2         #525
  angle = (midpoint - centerOfImage) * degreesPerPixel
  # print ("targetWidth: ", targetWidth)
  return angle
  
#YRes = 480    
videoStream = Camera().start()  # resolution found with    videoStream.camera.resolution[0]

networkTablesServer = 'roborio-1987-frc.local'
NetworkTables.initialize(server=networkTablesServer)
visionTable = NetworkTables.getTable('SmartDashboard')

hue = [50, 150]
sat = [30, 255]
val = [30, 255]

#constants found by measuring target
targetWidthInPixels = 144
#heightInPixels  =  1200
targetDistanceInInches = 100
targetWidthInInches = 15
#targetHeightInInches = 88
focalLength = (targetWidthInPixels*targetDistanceInInches)/targetWidthInInches

#calculations that get re-used
#targetWidthTimesFocalLength = targetWidthInInches * focalLength

#horizontalFOV = 62.2
#horizontalFOV = 2 * math.atan(targetWidthInPixels / (2 * focalLength));
horizontalFOV = 20
#verticalFOV = 48.8
degreesPerPixel = horizontalFOV / videoStream.camera.resolution[0]
centerOfImage = videoStream.camera.resolution[0] / 2

arguments = sys.argv            #index 0 is name of file, 1 is debug parameter
numOfArgs = len(arguments) 
debugMode = False
if numOfArgs > 1:
  if arguments[1] == "debug": 
    debugMode = True

while videoStream.read() is None: #wait until videostream isn't empty
  time.sleep(0.1)

while True:
  startTime = time.time()
  currentFrame = videoStream.read()
  currentTimeStamp = videoStream.getTimeStamp()
  filteredHSV = getHSVImage(currentFrame, hue, sat, val)
  contours = findContours(filteredHSV)
  boundingRects = findBoundingRects(contours, 25, 600, 25, 400)
  distance = None
  angle = None
  if len(boundingRects) > 0:
    target = filterTargetRects(boundingRects)
    visionTable.putBoolean('hasTarget', (target is not None)) 
    if target is not None:
      distance = distanceToTarget(target[1])
      angle = angleToTarget(target[0], target[2])
      timeDelta = time.perf_counter() - currentTimeStamp
      visionTable.putNumber("timeDelta", timeDelta)
      visionTable.putNumber('angle', angle)
      visionTable.putNumber('distance', distance)
  else:
    visionTable.putBoolean('hasTarget', False)
  
  if debugMode == True:
    # cv2.imshow("HSV", filteredHSV)
    cv2.imshow("RGB", currentFrame)
    endTime = time.time()
    elapsedTime = endTime - startTime
    fps = 1 / elapsedTime
    print ("FPS: ", fps)
    print ("Angle: ", angle)
  key = cv2.waitKey(1) & 0xFF
