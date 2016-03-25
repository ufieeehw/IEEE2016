#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Bool, Int8, Header
from geometry_msgs.msg import Pose, Point32, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3, PointStamped, Point

import cv2

def nothing(x):
    pass

class CameraAdjuster():
    def __init__(self):
        rospy.init_node("cam_adjuster")
        self.point_pub = rospy.Publisher("/act_block",PointStamped,queue_size=10)
        self.tf_broad = tf.TransformBroadcaster()

        cv2.namedWindow('tf')
        cv2.resizeWindow("tf", 1000,1)
        cv2.createTrackbar('x(mm)','tf',500,1000,nothing)
        cv2.createTrackbar('y(mm)','tf',500,1000,nothing)
        cv2.createTrackbar('z(mm)','tf',500,1000,nothing)
        cv2.createTrackbar('rx(mr)','tf',500,1000,nothing)
        cv2.createTrackbar('ry(mr)','tf',500,1000,nothing)
        cv2.createTrackbar('rz(mr)','tf',500,1000,nothing)

        r = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            cv2.waitKey(1)
            self.publish_known_points()
            self.get_tf()
            r.sleep()

    def get_tf(self):
        x = (cv2.getTrackbarPos('x(mm)','tf')-500) / 1000.0
        y = (cv2.getTrackbarPos('y(mm)','tf')-500) / 1000.0
        z = (cv2.getTrackbarPos('z(mm)','tf')-500) / 1000.0
        rx = (cv2.getTrackbarPos('rx(mr)','tf')-500) / 1000.0
        ry = (cv2.getTrackbarPos('ry(mr)','tf')-500) / 1000.0
        rz = (cv2.getTrackbarPos('rz(mr)','tf')-500) / 1000.0
        print (x,y,z),(rx,ry,rz)
        self.tf_broad.sendTransform((x,y,z), 
                        tf.transformations.quaternion_from_euler(rx,ry,rz),
                        rospy.Time.now(), "cam_1_adjust", "elevator")

    def publish_known_points(self):
        point = [.175,2.174,.196]
        p = Point(x=point[0], y=point[1], z=point[2])
        h = Header(stamp=rospy.Time.now(), frame_id="map")
        self.point_pub.publish(header=h, point=p)
        point = [.49,2.174,.2315]
        p = Point(x=point[0], y=point[1], z=point[2])
        h = Header(stamp=rospy.Time.now(), frame_id="map")
        self.point_pub.publish(header=h, point=p)


if __name__ == "__main__":
    c = CameraAdjuster()