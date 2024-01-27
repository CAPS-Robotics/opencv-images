#!pip install opencv-python

import cv2
import apriltag

image = cv2.imread('36h11_challenge.png')
gray = cv2.cvtColor(image, cv.COLOR_BGR2GRAY)

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(gray)

for i in range(len(results)):
  print(results[i].tag_id,results[i].center)
