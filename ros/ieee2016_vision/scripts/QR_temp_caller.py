#!/usr/bin/python
import rospy
import QR
import cv2
import numpy as np



# Make detector object. 12 is the number of QR codes to look for, 15 is the timeout in seconds
detector = QR.DetectQRCode(12,15, "camera/cam_1")

#Start processing. The ros node will update the images it searchs for
detected_frames = detector.begin_processing()

color = (0,0,0)
image = detector.image
del detector

for i in detected_frames:
    # BGR
    if i[0] == 'red': color = (0,0,255)
    if i[0] == 'blue': color = (255,0,0)
    if i[0] == 'green': color = (0,255,0)
    if i[0] == 'yellow': color = (0,255,255)

    cv2.circle(image,i[1],5,color,-1)

cv2.imshow('final',image)
cv2.waitKey(0)
cv2.destroyAllWindows()