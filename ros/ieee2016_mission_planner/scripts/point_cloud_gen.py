#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import Point32

import random

colors = ["blue","blue","blue","blue","red","red","red","red","yellow","yellow","yellow","yellow","green","green","green","green"]
coordinates = [(0,127,0),(0,127,3.81),(6.35,127,0),(6.35,127,3.81),(6.35*2,127,0),(6.35*2,127,3.81),(6.35*3,127,0),(6.35*3,127,3.81),(6.35*4,127,0),(6.35*4,127,3.81),(6.35*5,127,0),(6.35*5,127,3.81),(6.35*6,127,0),(6.35*6,127,3.81),(6.35*7,127,0),(6.35*7,127,3.81)]
found_points = []
for p in range(15):
    color = random.randint(0,len(colors)-1)
    coor = random.randint(0,len(coordinates)-1)
    found_points.append([colors[color],coordinates[coor]])
    del colors[color]
    del coordinates[coor]

print found_points
points = []
channels = [[],[],[]]
for p in found_points:
    if p[0] == "blue":
        channels[0].append(0) #R
        channels[1].append(0) #G
        channels[2].append(1) #B
    elif p[0] == "red":
        channels[0].append(1) #R
        channels[1].append(0) #G
        channels[2].append(0) #B
    elif p[0] == "green":
        channels[0].append(0) #R
        channels[1].append(1) #G
        channels[2].append(0) #B
    elif p[0] == "yellow":
        channels[0].append(1) #R
        channels[1].append(1) #G
        channels[2].append(0) #B

    points.append(Point32(
            x=p[1][0]/100.0 + random.uniform(-.005, .005),
            y=p[1][1]/100.0 + random.uniform(-.005, .005),
            z=p[1][2]/100.0 + random.uniform(-.005, .005)
        )
    )
rospy.init_node('temp')
point_pub = rospy.Publisher("/blocks", PointCloud, queue_size=1)

rgb_channels = [ChannelFloat32(name="r", values=channels[0]),
                ChannelFloat32(name="g", values=channels[1]),
                ChannelFloat32(name="b", values=channels[2])]

r = rospy.Rate(1)
while not rospy.is_shutdown():
    print "Publishing..."
    point_pub.publish(PointCloud(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="base_link"
                ),
            points=points,
            channels=rgb_channels
        )
    )
    r.sleep()