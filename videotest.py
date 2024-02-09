import cv2 as cv
import time  
import apriltag
import math
  
# define a video capture object 
vid = cv.VideoCapture(0) 
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
    else:
      #print(detect)
      for tag in detect:
        
        image = plotPoint(image, tag.center, CENTER_COLOR)
        
        # angle
        c1 = tag.corners[0]
        c2 = tag.corners[3] # side 1 is abs(p2-p1)
        c3 = tag.corners[1] # side 2 is abs(p4-p3)
        c4 = tag.corners[2]

        s1 = abs(c2[1]-c1[1])
        s2 = abs(c4[1]-c3[1])

        val = s1/s2
        
        # sides value to angle
        ang = -255.5*val+350.7
        
        # area
        # A = abs(math.ceil(abs(c1[0]-c2[0])*4)*math.ceil(abs(c4[1]-c1[1])*4))
        
        d1 = tag.corners[0]
        d2 = tag.corners[1] # distance needs it diff from angles
        d3 = tag.corners[2] # clockwise order
        d4 = tag.corners[3]
        
        A = ((d1[0]*d2[1]-d2[0]*d1[1])+
             (d2[0]*d3[1]-d3[0]*d2[1])+
             (d3[0]*d4[1]-d4[0]*d3[1])+
             (d4[0]*d1[1]-d1[0]*d4[1]))/2
        # area to dist
        
        f = 20 # focal len, next try 60
        
        # calibrate values
        f_x = 315.48326733
        f_y = 338.89718663
        c_x = 323.39820746
        c_y = 315.32874766
        
        pxmm = ((f_x+f_y)/2)/f # average fx fy, then divide by focal len to get the pxmm
        
        #print(f,A,pxmm)
        dist = 161.5 * f / (A/pxmm) # in mm
        dist = 19.19*dist**0.5 # if the tag is straight on decimal digit accuracy
        
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
        
        print(f"tag {tag.tag_id} with angle: {ang} with distance: {dist}")
      
    cv.imshow('frame', image) 
      
    # the 'q' button is set as the 
    # quitting button you may use any 
    # desired button of your choice 
    if cv.waitKey(140) & 0xFF == ord('i'): 
        img_c +=1
        cv.imwrite(f"i{img_c}.jpg", image)
        print("saved")
    if cv.waitKey(140) & 0xFF == ord('q'): 
        break
    if ret==False:
      break
  
# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv.destroyAllWindows() 
