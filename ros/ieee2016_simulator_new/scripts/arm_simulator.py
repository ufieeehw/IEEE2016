#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((1,127,3), np.uint8)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Z','image',0,127,nothing)
cv2.createTrackbar('X','image',0,127,nothing)

# ROS inits
rospy.init_node("arm_sim")
elevator = rospy.Publisher("/robot/arms/elevator", Int32, queue_size=2)
rail = rospy.Publisher("/robot/arms/rail", Int32, queue_size=2)


while not rospy.is_shutdown():
    cv2.imshow('image',img)
    cv2.waitKey(1)

    # get current positions of four trackbars
    z = cv2.getTrackbarPos('Z','image') #mm
    x = cv2.getTrackbarPos('X','image') #mm

    elevator.publish(Int32(data=z))
    rail.publish(Int32(data=x))

cv2.destroyAllWindows()