#! /usr/bin/env python2
import os
import thread
import time
import yaml

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Point
from ieee2016_msgs.srv import CameraSet
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Header
import tf


class PointProjector():
    '''
    The goal of the Point Projector is to display 3d points in the map frame as 2d points
    on the camera image.

    Takes a Point Stamped ros message or just 

    This is a major hypothetical program since it is hard to test it (I haven't tested it at all).
    '''
    def __init__(self, camera):
        cam_name = camera.name
        rospy.Subscriber('/camera/'+cam_name+'/project_points', PointStamped, self.got_point)
        self.camera = camera
        self.projection_matrix = self.camera.proj_mat

        # Will contain [[frame_id,[x,y,z]],[frame_id,[x,y,z]], ...]
        self.points = []

    def got_point(self, msg):
        # Handles getting ROS points.
        frame_id = msg.header.frame_id
        point = np.array([msg.point.x,msg.point.y,msg.point.z])
        
        self.points.append([frame_id,point])

    def draw_point(self, point):
        # Handles getting user inputted points.
        point = np.array([point])

    def draw_on_frame(self):
        # Method to take current camera frame and draw the points in the list on the frame
        current_frame = self.camera.image
        colors = [(0,0,255),(255,255,255),(255,0,0)]
        for i,point_stamped in enumerate(self.points):
            frame_id = point_stamped[0]
            point = point_stamped[1]
            
            # New point is [x,y,z,1] in the camera frame
            point = np.append(self.camera.transform_point(point, from_frame=frame_id, target_frame=self.camera.perspective_frame_id),1)
            if point[2] <= 0:
                # If the point is behind the camera, don't draw it.
                continue
            
            point_uvw = self.projection_matrix.dot(point)
            point_xy = (point_uv/point_uv[2])[:2]

            # The x,y points may need to be reversed.
            cv2.circle(current_frame,point_xy, 5, colors[i%len(colors)], -1)

class Camera():
    def __init__(self, cam_number):
        self.name = "cam_" + str(cam_number)

        # Two TF frames. Perspective is used for drawing points in the map frame.
        # Position can give the position and yaw of the camera.
        self.perspective_frame_id = self.name + "_vision"
        self.position_frame_id = self.name + "_pose"

        image_topic = "/camera/cam_stream"

        rospy.Subscriber(image_topic, Image, self.got_image)
        self.tf_listener = tf.TransformListener()

        # These will be filled in when the camera is activated
        self.proj_mat = None
        self.image = None
        self.active = False

        print self.name

    def __repr__(self):
        return self.name

    def activate(self):
        default_timeout = 2
        ret = rospy.ServiceProxy('/camera/camera_set', CameraSet)(String(data = self.name))
        self.proj_mat = np.array([ret.cam_info.P]).reshape((3, 4))
        self.proj_mat_pinv = np.linalg.pinv(self.proj_mat)

        # Wait until an image is loaded to continue (or timeout)
        start_time = time.time()
        while self.image is None and time.time() - start_time < default_timeout and not rospy.is_shutdown():
            time.sleep(.1)

        self.active = True

    def deactivate(self):
        ret = rospy.ServiceProxy('/camera/camera_set', CameraSet)(String(data = "STOP"))

        self.active = False

    def get_tf(self, mode = "pose", target_frame = "base_link", time = None):
        # Returns the relative [x,y,yaw] between the target_frame and the camera
        # Mode can be 'pose' for the pose tf frame, or 'vision' for the vision frame at 'time'. Default is 'pose'.
        if mode == "pose":
            if time is None: time = self.tf_listener.getLatestCommonTime(target_frame, self.position_frame_id)
            self.tf_listener.waitForTransform(target_frame, self.position_frame_id, time, rospy.Duration(1.0))
            pos, quaternion = self.tf_listener.lookupTransform(target_frame, self.position_frame_id, time)
            rot = tf.transformations.euler_from_quaternion(quaternion)
        elif mode == "vision":
            if time is None: time = self.tf_listener.getLatestCommonTime(target_frame, self.perspective_frame_id)
            self.tf_listener.waitForTransform(target_frame, self.position_frame_id, time, rospy.Duration(1.0))
            pos, quaternion = self.tf_listener.lookupTransform(target_frame, self.perspective_frame_id, time)
            rot = tf.transformations.euler_from_quaternion(quaternion)

        return np.array([pos[0], pos[1], rot[2]])

    def transform_point(self, point, from_frame=None, target_frame="map", time=None):
        # Given a 3d point in the camera frame, return that point in the map frame.
        if from_frame is None: from_frame = self.perspective_frame_id
        if time is None: time = self.tf_listener.getLatestCommonTime(target_frame, from_frame)
        self.tf_listener.waitForTransform(target_frame, from_frame, time, rospy.Duration(1.0))
        p_s = PointStamped(
                header = Header(
                        stamp = time,
                        frame_id = from_frame
                    ),
                point = Point(
                        x = point[0],
                        y = point[1],
                        z = point[2]
                    )
            )
        new_point = self.tf_listener.transformPoint(target_frame, p_s)
        return np.array([new_point.point.x, new_point.point.y, new_point.point.z])

    def make_3d_point(self, point_1, point_2, distance_between, output_frame = None):
        # Given two planar points [u,v] and the real life distance between them (m), return the 3d camera frame coordiantes of those points.
        # Returns [[x_1,y_1,z_1],[x_2,y_2,z_2]] in whatever the output_frame is

        # Genereate projection rays through points 1 and 2
        ray_1 = self.make_3d_ray(point_1)
        ray_2 = self.make_3d_ray(point_2)

        if ray_1 == False: return False

        # Angle between two rays
        mag_1, mag_2 = np.sqrt(ray_1.dot(ray_1)), np.sqrt(ray_2.dot(ray_2))
        theta = np.arccos(np.dot(ray_1, ray_2) / (mag_1 * mag_2))
        dist = (distance_between / 2.0) / (np.arctan(theta / 2.0))

        points = np.array([ray_1[:3] * dist / np.linalg.norm(ray_1[:3]), ray_2[:3] * dist / np.linalg.norm(ray_2[:3])])

        # Transform the output points to the appropriate frame
        if output_frame is None: return points
        new_frame_points = []
        for p in points:
            new_frame_points.append(self.transform_point(p, output_frame))
        return np.array(new_frame_points)

    def make_3d_point(self, vector, dist, output_frame = None, time = None):
        # Given a vector through a point and a real world distance to that point from the camera
        # make return [x,y,z] of that point in the output_frame (default is the camera frame)
        point = vector * dist / np.linalg.norm(vector)

        if output_frame is None: return point
        return self.transform_point(point, target_frame = output_frame, time = time)

    def make_3d_vector(self, point):
        # Given a point in the camera frame, make a 3d vector pointing toward the point that intersects with
        # the object in the real world at some distance.
        if self.proj_mat is None: return False

        # Genereate projection ray through point
        vect = self.proj_mat_pinv.dot(np.append(point, 1).reshape((3, 1))).reshape((1, 4))[0][:3]
        return np.array(vect)

    def got_image(self, msg):
        try:
            self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

class CameraManager():
    def __init__(self):
        # ROS inits
        # self.cam_1_pub = rospy.Publisher("/camera/cam_1", Image, queue_size = 1)
        # self.cam_2_pub = rospy.Publisher("/camera/cam_2", Image, queue_size = 1)
        self.cam_pub = rospy.Publisher("/camera/cam_stream", Image, queue_size = 1)
        
        rospy.init_node("camera_manager")
        br = CvBridge()
        rospy.Service('/camera/camera_set', CameraSet, self.set_camera)

        # Find the cameras with the given parameters
        self.cam_1 = cv2.VideoCapture(rospy.get_param("~cam_1_index"))
        self.cam_1.set(3, rospy.get_param("~cam_1_width"))   # CV_CAP_PROP_FRAME_WIDTH
        self.cam_1.set(4, rospy.get_param("~cam_1_heigth"))  # CV_CAP_PROP_FRAME_HEIGHT
        self.cam_1.set(5, rospy.get_param("~fps"))           # FPS

        self.cam_2 = cv2.VideoCapture(rospy.get_param("~cam_2_index"))
        self.cam_2.set(3, rospy.get_param("~cam_2_width"))   # CV_CAP_PROP_FRAME_WIDTH
        self.cam_2.set(4, rospy.get_param("~cam_2_heigth"))  # CV_CAP_PROP_FRAME_HEIGHT
        self.cam_2.set(5, rospy.get_param("~fps"))           # FPS

        self.cam = None

        print "> Initialization Complete."
        rate = rospy.Rate(rospy.get_param("~fps"))  # hz
        while not rospy.is_shutdown():
            try:
                if self.cam:
                    self.cam_pub.publish(br.cv2_to_imgmsg(self.cam.read()[1], "bgr8"))
                rate.sleep()
            except:
                print "> Error opening Camera:", self.cam
                rate.sleep()

    def set_camera(self, srv):
        cam_name = srv.cam_name.data
        self.cam = None
        if cam_name == "cam_1":
            print "> Publishing Camera 1."
            self.cam = self.cam_1
        elif cam_name == "cam_2":
            print "> Publishing Camera 2."
            self.cam = self.cam_2
        elif cam_name == "STOP":
            print "> Stopping Publishing."
            self.cam = None
            return CameraInfo()

        return self.get_cam_info(cam_name)

    def get_cam_info(self, cam_name):
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
