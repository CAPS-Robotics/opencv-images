import cv2 as cv
import time  
import apriltag
import math
import json
import keyboard
import time
import wpilib
from networktables import NetworkTables as nt
import logging

# network tables
logging.basicConfig(level=logging.DEBUG)

nt.initialize(server="roborio-2410-frc.local")
sd = nt.getTable("SmartDashboard")
  
# define a video capture object 
vid = cv.VideoCapture(0) 

# vid.set(3, 1920)
# vid.set(4, 1080)

vid.set(3, 1280)
vid.set(4, 720)

vid.set(cv.CAP_PROP_FPS, 30)

print(vid)

if not vid.isOpened():
    raise IOError("Cannot open")
    
CENTER_COLOR = (203,192,255)
CORNER_COLOR = (50.205,50)
LL = 5 # LINE_LENGTH

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv.putText(image, str(text), center, cv.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)


def plotPoint(image, center, color):
	center = (int(center[0]), int(center[1]))
	image = cv.line(image,
	(center[0] - LL, center[1]),
	(center[0] + LL, center[1]),
	color,
	3)
	image = cv.line(image,
	(center[0], center[1] - LL),
	(center[0], center[1] + LL),
	color,
	3)
	return image

img_c = 0

angle_norm = {
1: [0,0,0,0,0],
2: [0,0,0,0,0],
6: [0,0,0,0,0],
7: [0,0,0,0,0],
8: [0,0,0,0,0],
9: [0,0,0,0,0],
10: [0,0,0,0,0],
14: [0,0,0,0,0],
15: [0,0,0,0,0],
16: [0,0,0,0,0],
}

while(True): 
    # Capture the video frame 
    # by frame 
    ret, frame = vid.read()
    image=frame
    #print(ret, frame) 
    
    #frame = cv2.imread("/home/pi/testtag.jpg")
  
    # Display the resulting frame 
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    h, w = frame.shape[:2]
    h,w = int(h),int(w)
    r_h,r_w = h,w
    resize = cv.resize(gray, (w,h))
    detect = detector.detect(resize)
	
    if not detect:
      #print("Nothing")
      image = frame
      image = cv.line(image, [int(w/2),0], [int(w/2), h], (0, 255, 0), 3)
      
      info = {"id":"none","angle":"N/A","dist":"N/A","(s1/s2)/dist":"N/A"}
      print(json.dumps(info))
      
      sd.putNumber("id", -1)
      sd.putNumber("angle", 1000)
      sd.putNumber("dist", -1)
      sd.putNumber("aprilDist", -1)
    else:
      #print(detect)
      for tag in detect:
        
        image = plotPoint(image, tag.center, CENTER_COLOR)
        image = cv.line(image, [int(w/2),0], [int(w/2), h], (0, 255, 0), 3)
        
        d1 = tag.corners[0]
        d2 = tag.corners[1] # distance needs it diff from angles
        d3 = tag.corners[2] # clockwise order
        d4 = tag.corners[3]
        
        # angle
        s1 = math.dist(d1,d2)
        s2 = math.dist(d2,d3)
        s3 = math.dist(d3,d4)
        s4 = math.dist(d4,d1)

        val = (s1+s4)/(s2+s3)
        
        # sides value to angle
        ang = -1964*val+2034
        
        ang = round(float("%.1f"%(ang/10))*2)/2*10
        
        # area
        A = ((d1[0]*d2[1]-d2[0]*d1[1])+
             (d2[0]*d3[1]-d3[0]*d2[1])+
             (d3[0]*d4[1]-d4[0]*d3[1])+
             (d4[0]*d1[1]-d1[0]*d4[1]))/2
        # area to dist
        
        f = 60 # focal len, next try 60
        
        # calibrate values
        f_x = 706.6876743 #315.48326733
        f_y = 713.88098602 #338.89718663 
        
        pxmm = ((f_x+f_y)/2)/f # average fx fy, then divide by focal len to get the pxmm
        
        #print(f,A,pxmm)
        dist = 161.5 * f / (A/pxmm) # in mm
        dist = 1.063*dist+52.06 # if the tag is straight on decimal digit accuracy
        
        # corner marks
        for corner in tag.corners:
          image = plotPoint(image, corner, CORNER_COLOR)
        
        # id
        image = plotText(image, tag.center, CENTER_COLOR, str(tag.tag_id))
          
        if len(detect)>1:
          c0 = detect[0].center
          c1 = detect[1].center
          c0, c1 = tuple([int(c0[0]),int(c0[1])]),tuple([int(c1[0]),int(c1[1])])
          image = cv.line(image, c0, c1, (0, 255, 0), 3)
          midp = (int(c0[0]+c1[0])/2,int(c0[1]+c1[1])/2)
          image = plotPoint(image, midp, CENTER_COLOR)
          #print(c0,c1)
          
        info = {"id":tag.tag_id,"angle":ang,"dist":dist,"val":val}
        print(json.dumps(info))
        
        sd.putNumber("id", tag.tag_id)
        sd.putNumber("angle", ang)
        sd.putNumber("dist", dist)
        sd.putNumber("aprilDist", val/dist)
      
    #cv.imshow('frame', image) 
    if ret==False:
      break
  
# After the loop release the cap object 
vid.release() 

def transpose(array,val):
    # shift element forward
    for i in range(len(array)):
        if i!= len(array)-1:
            array[i] = array[i+1]

    # set last value        
    array[-1]=val

    return array
