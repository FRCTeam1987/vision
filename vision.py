from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import imutils
import time

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
  def start(self):
    t = Thread(target = self.update, args = ())
    t.daemon = True
    t.start()
    return self
  def update(self):
    for f in self.stream:
      self.frame = f.array
      self.rawCapture.truncate(0)
      if self.stopped:
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()
        return
  def read(self):
    return self.frame
  def stop(self):
    self.stopped = True
    
def getHSVImage(RGBImage, hue, sat, val):
    HSVImage = cv2.cvtColor(RGBImage, cv2.COLOR_BGR2HSV)
    return cv2.inRange(HSVImage, (hue[0], sat[0], val[0]), (hue[1], sat[1], val[1]))
    
def findContours(HSVImage):
    im1, contours, hierarchy = cv2.findContours(HSVImage, mode = cv2.RETR_EXTERNAL, method = cv2.CHAIN_APPROX_SIMPLE)
    return contours
    
def getContourImage(HSVImage, contours):
    return cv2.drawContours(HSVImage, contours, -1, (255, 255, 0), 4)
    
def filterContours(contours, minWidth, maxWidth, minHeight, maxHeight):
    filteredContours = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)    #duplicate calculation?
        if w >= minWidth and w <= maxWidth and h >= minHeight and h <= maxHeight:
            filteredContours.append(contour)
    return filteredContours
    
def findBoundingRects(contours, minWidth, maxWidth, minHeight, maxHeight):
    filteredRects = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if w >= minWidth and w <= maxWidth and h >= minHeight and h <= maxHeight:
            filteredRects.append((x,y,w,h))
    return (filteredRects)
    
def distanceToTarget(targetWidth):
    return targetWidthTimesFocalLength / targetWidth
    
videoStream = Camera().start()  # resolution found with    videoStream.camera.resolution[0]

hue = [50, 150]
sat = [30, 255]
val = [30, 255]

#constants found by measuring target
targetWidthInPixels = 190
targetDistanceInInches = 136
targetWidthInInches = 20
focalLength = (targetWidthInPixels*targetDistanceInInches)/targetWidthInInches

#calculations that get re-used
targetWidthTimesFocalLength = targetWidthInInches * focalLength

while videoStream.read() is None: #wait until videostream isn't empty
  time.sleep(0.1)

while True:
  currentFrame = videoStream.read()
  filteredHSV = getHSVImage(currentFrame, hue, sat, val)
  contours = findContours(filteredHSV)
  filteredContours = filterContours(contours, 25, 600, 25, 400)
  boundingRects = findBoundingRects(filteredContours, 25, 600, 25, 400)
  distance = None
  if len(boundingRects) > 0:
    distance = distanceToTarget(boundingRects[0][2]) #[0][2] is first width in boundingRects array, later change to find actual target. first number is rectangle index, second is width
  
  print(distance)
  print(boundingRects) 
  
  #cv2.imshow("HSV", filteredHSV)
  cv2.imshow("Contours", getContourImage(filteredHSV, filteredContours))
  #cv2.imshow("Frame", currentFrame)
  
  key = cv2.waitKey(1) & 0xFF
  
