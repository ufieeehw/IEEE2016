#! /usr/bin/env python2
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from cv_bridge import CvBridge, CvBridgeError
import tf


from ieee2016_msgs.srv import CameraSet

import cv2
import numpy as np
import time

# These are the upper and lower boundaries for each color
# These WILL be replaced by an automatically generated calibration file
LOWER_RED = np.array([0, 180, 180])
UPPER_RED = np.array([20, 255, 255])
LOWER_GREEN = np.array([50, 40, 40])
UPPER_GREEN = np.array([80, 250, 250])
LOWER_BLUE = np.array([80, 80, 80])
UPPER_BLUE = np.array([150, 255, 255])
LOWER_YELLOW = np.array([20, 120, 120])
UPPER_YELLOW = np.array([50, 255, 255])

def nothing(x):
    # Just for slider bars
    pass

def make_2D_rotation_mat(angle):
    c, s = np.cos(angle), np.sin(angle)
    mat = np.array([
        [c,     -s],
        [s,      c],
    ],
    dtype=np.float32)
    return mat

class VisualServo():
    def __init__(self):
        self.twist_pub = rospy.Publisher("/robot/twist", TwistStamped, queue_size=2)
        self.tf_listener = tf.TransformListener()
        
        self.frame = "/cam_1_pose"
        self.tf_listener.waitForTransform("base_link", self.frame, rospy.Time(0), rospy.Duration(5.0))
        print " > TF link established."


        self.img_sub = rospy.Subscriber("/camera/cam_1", Image, self.image_recieved)
        self.cam_info = rospy.ServiceProxy('/camera/camera_set', CameraSet)(String(data="cam_1"))

        self.image = np.zeros((500,500,3), np.float32)
        self.hsv = self.image

        #sliders = np.zeros((10,500,3), np.float32)
        # cv2.namedWindow('sliders')

        # cv2.createTrackbar('H_L','sliders',0,255,nothing)
        # cv2.createTrackbar('H_U','sliders',0,255,nothing)
        # cv2.createTrackbar('S_L','sliders',0,255,nothing)
        # cv2.createTrackbar('S_U','sliders',0,255,nothing)
        # cv2.createTrackbar('V_L','sliders',0,255,nothing)
        # cv2.createTrackbar('V_U','sliders',0,255,nothing)
        #115,82,26
        #time.sleep(5)

        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            img_copy = np.copy(self.image)
            # Convert BGR to HSV
            hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
            hsv = cv2.bilateralFilter(hsv,3,75,75)

            #cv2.imshow('sliders',sliders)

            # define range color in HSV
            #lower = np.array([cv2.getTrackbarPos('H_L','sliders'),cv2.getTrackbarPos('S_L','sliders'),cv2.getTrackbarPos('V_L','sliders')])
            #upper = np.array([cv2.getTrackbarPos('H_U','sliders'),cv2.getTrackbarPos('S_U','sliders'),cv2.getTrackbarPos('V_U','sliders')])
            lower = np.array([100,0,0])
            upper = np.array([130,255,255])

            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((15,15),np.float32)/25
            mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(img_copy,img_copy, mask= mask)

            contours,hierarchy = cv2.findContours(mask_opened, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            #print contours
            if len(contours) == 0: continue
            k = cv2.waitKey(50) & 0xFF

            largest_cnt = contours[0]

            solidity_threshold = .75

            for i,c in enumerate(contours):
                # Test solidity of shape
                area = cv2.contourArea(c)
                hull = cv2.convexHull(c)
                hull_area = cv2.contourArea(hull)
                solidity = float(area)/hull_area

                # We check for the largest patch of blue that does not contain any other contours inside it (this will happen on the side of a block)
                if cv2.contourArea(c) >= cv2.contourArea(largest_cnt): #and hierarchy[0][i][2] == -1 and solidity > solidity_threshold:
                    largest_cnt = c
        
                x,y,w,h = cv2.boundingRect(largest_cnt)
                cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,0,255),2)

                # LPF for the mid point so we dont get sudden jumps
                #if mid_point == None: mid_point = x+w/2.0
                mid_point = (x+w/2.0)#mid_point*.85 + .15*(x+w/2.0)
                delta = img_copy.shape[1]/2-mid_point

                #print "Delta:",delta
                if abs(delta) <= tolerance:
                    print "Done"
                    #self.img_sub.unregister()
                    #break

                self.publish_twist(delta/10000.0)

                cv2.line(img_copy,(int(mid_point),0),(int(mid_point),img_copy.shape[0]), (0,0,0), 2)
                cv2.line(img_copy,(img_copy.shape[1]/2,0),(img_copy.shape[1]/2,img_copy.shape[0]), (0,255,0), 2)



                #print approx
                cv2.drawContours(img_copy, largest_cnt, -1, (255,0,0), 3)
            cv2.imshow('frame',img_copy)
            #cv2.imshow('mask',mask)
            #cv2.imshow('res',res)


    def image_recieved(self,msg):
        rospy.loginfo("Frame updated")
        try:
            self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e
    
    def publish_twist(self, frame_twist_y):
        # Given a twist in a camera frame, cacluate that twist in the base frame
        t = self.tf_listener.getLatestCommonTime("/base_link", self.frame)
        _, quaternion = self.tf_listener.lookupTransform("/base_link",self.frame, t)
        camera_rotation = tf.transformations.euler_from_quaternion(quaternion)

        raw_vel = np.array([0,frame_twist_y])
        des_vel = np.dot(make_2D_rotation_mat(camera_rotation[2]),raw_vel)

        # Publish that twist
        t_s = TwistStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id="base_link"
                ),
                twist=Twist(
                        linear=Vector3(
                                x=des_vel[0],
                                y=des_vel[1],
                                z=0
                            ),
                        angular=Vector3(
                                x=0,
                                y=0,
                                z=0
                            )
                    )

            )
        self.twist_pub.publish(t_s)


rospy.init_node('visual_servo')
v = VisualServo()
