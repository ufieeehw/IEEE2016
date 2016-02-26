#! /usr/bin/env python2
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
import tf

from ieee2016_msgs.srv import CameraSet

import cv2
import numpy as np
import time
import thread
import yaml
import os

class Camera():
    def __init__(self, camera_name):
        self.camera_name = camera_name
        self.frame_id = camera_name

        image_topic = "/camera/"+camera_name
        rospy.Subscriber(image_topic, Image, self.got_image)
        self.tf_listener = tf.TransformListener()

        # These will be filled in when the camera is activated
        self.proj_mat = None
        self.image = None
        self.active = False

    def __repr__(self):
        return self.camera_name

    def activate(self):
        default_timeout = 2
        ret = rospy.ServiceProxy('/camera/camera_set', CameraSet)(String(data=self.camera_name))
        self.proj_mat = np.array([ret.cam_info.P]).reshape((3,4))
        
        # Wait until an image is loaded to continue (or timeout)
        start_time = time.time()
        while self.image == None and time.time() - start_time < default_timeout and not rospy.is_shutdown():
            time.sleep(.1)

        self.active = True

    def deactivate(self):
        ret = rospy.ServiceProxy('/camera/camera_set', CameraSet)(String(data="STOP"))
        self.active = False

    def get_tf(self, target_frame="base_link"):
        # Returns the relative [x,y,yaw] between the target_frame and the camera
        t = self.tf_listener.getLatestCommonTime(target_frame, self.frame_id)
        pos, quaternion = self.tf_listener.lookupTransform(target_frame,self.frame_id, t)
        rot = tf.transformations.euler_from_quaternion(quaternion)

        return np.array([pos[0],pos[1],rot[2]])
    
    def transform_point(self, point, target_frame="base_link"):
        # Given a point in the camera frame, return that point in the map frame.

        ''' UNTESTED AS OF RIGHT NOW '''

        t = self.tf_listener.getLatestCommonTime(target_frame, self.frame_id)
        p_s = PointStamped(
                header=Header(
                        stamp=t,
                        frame_id=self.frame_id
                    ),
                point=Point(
                        x=point[0],
                        y=point[1],
                        z=point[2]
                    )
            )
        new_point = tf_listener.transformPoint(target_frame,p_s)
        return np.array([new_point.point.x,new_point.point.y,new_point.point.z])

    def make_3d(self, point_1, point_2, distance_between):
        # Given two planar points [u,v] and the real life distance between them (m), return the 3d camera frame coordiantes of those points.
        # Returns [[x_1,y_1,z_1],[x_2,y_2,z_2]]

        ''' UNTESTED AS OF RIGHT NOW '''

        if self.proj_mat == None: return False

        proj_mat_pinv = np.linalg.pinv(np.array([self.proj_mat]).reshape((3,4)))

        # Genereate projection rays through points 1 and 2
        ray_1 = proj_mat_pinv.dot(np.append(point_1,1).reshape((3,1))).reshape((1,4))[0]
        ray_2 = proj_mat_pinv.dot(np.append(point_2,1).reshape((3,1))).reshape((1,4))[0]

        # Angle between two rays
        mag_1, mag_2 = np.sqrt(ray_1.dot(ray_1)), np.sqrt(ray_2.dot(ray_2))
        theta = np.arccos(np.dot(ray_1,ray_2)/(mag_1 * mag_2))    
        dist = (distance_between / 2.0) / (np.arctan(theta / 2.0))

        vect_1 = proj_mat_pinv.dot(np.append(point_1,1).reshape((3,1))).reshape((1,4))[0][:3]
        vect_2 = proj_mat_pinv.dot(np.append(point_2,1).reshape((3,1))).reshape((1,4))[0][:3]

        return np.array([vect_1 * dist / np.linalg.norm(vect), vect_2 * dist / np.linalg.norm(vect)])

    def got_image(self,msg):
        try:
            self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e


class CameraManager():
    def __init__(self):

        # ROS inits
        self.cam_1_pub = rospy.Publisher("/camera/cam_1", Image, queue_size=1)
        #self.cam_2_pub = rospy.Publisher("/camera/cam_2", Image, queue_size=1)
        rospy.init_node("camera_manager")
        br = CvBridge()
        rospy.Service('/camera/camera_set', CameraSet, self.set_camera)  

        # Find the cameras with the given parameters
        self.cam_1 = cv2.VideoCapture(rospy.get_param("~cam_1_index"))
        self.cam_1.set(3, rospy.get_param("~cam_1_width")) #CV_CAP_PROP_FRAME_WIDTH
        self.cam_1.set(4, rospy.get_param("~cam_1_heigth")) #CV_CAP_PROP_FRAME_HEIGHT

        #self.cam_2 = cv2.VideoCapture(rospy.get_param("~cam_2_index"))
        #self.cam_2.set(3, rospy.get_param("~cam_2_width")) #CV_CAP_PROP_FRAME_WIDTH
        #self.cam_2.set(4, rospy.get_param("~cam_2_heigth")) #CV_CAP_PROP_FRAME_HEIGHT

        self.cam = None
        self.pub = None
    
        print "> Initialization Complete."
        rate = rospy.Rate(10) #hz
        run_rate = rospy.Rate(30) #hz
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
            print "> Publishing Camera 1."
            self.cam = self.cam_1
            self.pub = self.cam_1_pub
        elif cam_name == "cam_2":
            print "> Publishing Camera 2."
            self.cam = self.cam_2
            self.pub = self.cam_2_pub
        elif cam_name == "STOP":
            print "> Stopping Publishing."
            self.cam = None
            self.pub = None
            return None

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

if __name__ == "__main__":
    c = CameraManager()
