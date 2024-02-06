import cv2 as cv
import time  
import apriltag
  
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
        print(tag.tag_id,-255.5*val+350.7)
        
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
      
    cv.imshow('frame', image) 
      
    # the 'q' button is set as the 
    # quitting button you may use any 
    # desired button of your choice 
    if cv.waitKey(140) & 0xFF == ord('q'): 
        break
    if ret==False:
      break
  
# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv.destroyAllWindows() 
