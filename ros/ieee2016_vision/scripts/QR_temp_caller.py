#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32,PointStamped
from std_msgs.msg import Header

import QR
import cv2
import numpy as np
import time
import struct


rospy.init_node('temp')
point_pub = rospy.Publisher("/arm/waypoints", PointCloud, queue_size=1)

# Make detector object. 8 is the number of QR codes to look for, 15 is the timeout in seconds
detector = QR.DetectQRCode(2,10, "cam_0")

#Start processing. The ros node will update the images it searchs for
detected_points = detector.begin_processing()
points = []
channels = [[],[],[]]
for p in detected_points:
    if p[0] == "blue":
        #rgbs.append("FF0000")#struct.pack('i', 0x0000ff))
        channels[0].append(0) #R
        channels[1].append(0) #G
        channels[2].append(1) #B
    elif p[0] == "red":
        #rgbs.append(struct.unpack('f', struct.pack('i', 0xff0000))[0])
        channels[0].append(1) #R
        channels[1].append(0) #G
        channels[2].append(0) #B
    elif p[0] == "green":
        #rgbs.append(struct.unpack('f', struct.pack('i', 0x00ff00))[0])
        channels[0].append(0) #R
        channels[1].append(1) #G
        channels[2].append(0) #B
    elif p[0] == "yellow":
        #rgbs.append(struct.unpack('f', struct.pack('i', 0xffff00))[0])
        channels[0].append(1) #R
        channels[1].append(1) #G
        channels[2].append(0) #B

    points.append(Point32(
            x=p[1][0]/100.0,
            y=p[1][1]/100.0,
            z=p[1][2]/100.0
        )
    )
rgb_channels = [ChannelFloat32(name="r", values=channels[0]),ChannelFloat32(name="g", values=channels[1]),ChannelFloat32(name="b", values=channels[2])]
r = rospy.Rate(10)
while not rospy.is_shutdown():
    print "Publishing points"
    point_pub.publish(PointCloud(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="camera"
                ),
            points=points,
            channels=rgb_channels
        )
    )
    r.sleep()