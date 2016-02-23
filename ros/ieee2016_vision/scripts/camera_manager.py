#! /usr/bin/env python2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ieee2016_msgs.srv import CameraSet

import cv2
import numpy as np
import time
import thread
import yaml
import os

class CameraManager():
    def __init__(self):
        # There may be a better way to init these cameras
        self.cam_1 = cv2.VideoCapture(1)
        self.cam_2 = cv2.VideoCapture(2)

        # ROS inits
        self.cam_1_pub = rospy.Publisher("/camera/cam_1", Image, queue_size=1)
        self.cam_2_pub = rospy.Publisher("/camera/cam_2", Image, queue_size=1)
        rospy.init_node("camera_manager")
        br = CvBridge()
        rospy.Service('/camera/camera_set', CameraSet, self.set_camera)  



        self.cam = None
        self.pub = None
    
        rate = rospy.Rate(10) #hz
        run_rate = rospy.Rate(50) #hz
        while not rospy.is_shutdown():
            if self.cam and self.pub: 
                # Aint no rest for the wicked
                self.pub.publish(br.cv2_to_imgmsg(self.cam.read()[1], "bgr8"))
                run_rate.sleep() # Okay some rest, just to not overload CPU
            else:
                rate.sleep()

    def set_camera(self, srv):
        cam_name = srv.cam_name.data
        if cam_name == "cam_1":
            print "> Publishing camera 1"
            self.cam = self.cam_1
            self.pub = self.cam_1_pub
        elif cam_name == "cam_2":
            print "> Publishing camera 2"
            self.cam = self.cam_2
            self.pub = self.cam_2_pub

        return self.get_cam_info(cam_name)

    def get_cam_info(self,cam_name):
        cam_name = "calibrations/" + cam_name + ".yaml"
        stream = file(os.path.join(os.path.dirname(__file__), cam_name), 'r')

        calib_data = yaml.load(stream)

        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

c = CameraManager()
