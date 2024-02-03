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
    #print(ret, frame) 
    
    #frame = cv2.imread("/home/pi/testtag.jpg")
  
    # Display the resulting frame 
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    h, w = frame.shape[:2]
    h,w = int(h/3.5),int(w/3.5)
    r_h,r_w = h,w
    resize = cv.resize(gray, (w,h))
    detect = detector.detect(resize)
	
    if not detect:
      #print("Nothing")
      image = frame
    else:
      for tag in detect:
        #print(f"Tag: {detect.tag_id} Center: {detect.center}")
        image = cv.cvtColor(resize, cv.COLOR_GRAY2BGR)
        image = plotPoint(image, tag.center, CENTER_COLOR)
        for corner in tag.corners:
          image = plotPoint(image, corner, CORNER_COLOR)
    
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
