#!/usr/bin/env python
## Math
import numpy as np
import math
## Ros/tools
import rospy
import tf
## Ros msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from ieee2016_msgs.msg import UltraSonicStamped, UltraSonicActivator
## Misc
import cv2

def nothing(x):
    pass

rospy.init_node("ultrasonic_simulator")
br = tf.TransformBroadcaster()
ultrasonic_pub = rospy.Publisher('/robot/navigation/ultra_sonic/sensor_data', UltraSonicStamped, queue_size=10)

sliders = np.zeros((10,500,3), np.float32)
cv2.namedWindow('sliders')

cv2.createTrackbar('us_1','sliders',0,1000,nothing)
cv2.createTrackbar('us_2','sliders',0,1000,nothing)
cv2.createTrackbar('us_3','sliders',0,1000,nothing)
cv2.createTrackbar('us_4','sliders',0,1000,nothing)


r = rospy.Rate(10)
while not rospy.is_shutdown():
    cv2.imshow('sliders',sliders)

    br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                       rospy.Time.now(), 'base_link', "map")

    br.sendTransform((.15, .1, 0), tf.transformations.quaternion_from_euler(0, 0, 1.5707),
                       rospy.Time.now(), 'us_1', "base_link")
    br.sendTransform((-.15, .1, 0), tf.transformations.quaternion_from_euler(0, 0, 1.5707),
                       rospy.Time.now(), 'us_2', "base_link")
    br.sendTransform((-.15, -.1, 0), tf.transformations.quaternion_from_euler(0, 0, -1.5707),
                       rospy.Time.now(), 'us_3', "base_link")
    br.sendTransform((.15, -.1, 0), tf.transformations.quaternion_from_euler(0, 0, -1.5707),
                       rospy.Time.now(), 'us_4', "base_link")
    for i in range(1,5):
        this_id = "us_"+ str(i)
        ultrasonic_pub.publish(UltraSonicStamped(
                        header=Header(
                            stamp=rospy.Time.now(),
                            frame_id=this_id
                        ),
                        range=cv2.getTrackbarPos(this_id,'sliders')/1000.0
        ))

    cv2.waitKey(1)
    r.sleep()