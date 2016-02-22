#! /usr/bin/env python2
import rospy
from sensor_msgs.msg import Image as Image_msg, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

colors = []

def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(frame[y,x].tolist())
        print len(colors)

def nothing(x):
    pass

class temp():
    def __init__(self):
        rospy.init_node('color_picker')
        rospy.Subscriber("/camera/image_raw",Image_msg,self.image_recieved)
        self.image = np.zeros((500,500,3), np.float32)
        self.hsv = self.image

        sliders = np.zeros((10,500,3), np.float32)
        cv2.namedWindow('sliders')

        cv2.createTrackbar('H_L','sliders',0,255,nothing)
        cv2.createTrackbar('H_U','sliders',0,255,nothing)
        cv2.createTrackbar('S_L','sliders',0,255,nothing)
        cv2.createTrackbar('S_U','sliders',0,255,nothing)
        cv2.createTrackbar('V_L','sliders',0,255,nothing)
        cv2.createTrackbar('V_U','sliders',0,255,nothing)
        #115,82,26

        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            img_copy = np.copy(self.image)
            # Convert BGR to HSV
            hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
            hsv = cv2.bilateralFilter(hsv,3,75,75)
            # Filtering

            cv2.imshow('sliders',sliders)


            # define range of blue color in HSV
            lower = np.array([cv2.getTrackbarPos('H_L','sliders'),cv2.getTrackbarPos('S_L','sliders'),cv2.getTrackbarPos('V_L','sliders')])
            upper = np.array([cv2.getTrackbarPos('H_U','sliders'),cv2.getTrackbarPos('S_U','sliders'),cv2.getTrackbarPos('V_U','sliders')])
            #lower = np.array([55,20,0])
            #upper = np.array([100,255,255])

            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((11,11),np.float32)/25
            mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(img_copy,img_copy, mask= mask)

            contours,hierarchy = cv2.findContours(mask_opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #print contours
            k = cv2.waitKey(50) & 0xFF
            if len(contours) == 0: continue
            largest_cnt = contours[0]
            for c in contours:
                if cv2.contourArea(c) > cv2.contourArea(largest_cnt): largest_cnt = c

            x,y,w,h = cv2.boundingRect(largest_cnt)
            cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,0,255),2)
            epsilon = 0.05*cv2.arcLength(largest_cnt,True)
            approx = cv2.approxPolyDP(largest_cnt,epsilon,True)
            #print approx
            cv2.drawContours(img_copy, largest_cnt, -1, (255,0,0), 3)
            cv2.drawContours(img_copy, approx, -1, (0,255,0), 3)

            cv2.imshow('frame',img_copy)
            #cv2.imshow('mask',mask)
            #cv2.imshow('res',res)

    def image_recieved(self,msg):
        #print "Frame updated"
        try:
            self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

        
t = temp()
