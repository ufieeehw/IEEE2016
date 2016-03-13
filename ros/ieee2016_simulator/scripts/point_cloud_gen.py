#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud,ChannelFloat32
from geometry_msgs.msg import Point32

import random

def generate_simulated_points(stage):
    x_mod, y_mod, z_mod = 0,0,0
    if stage == 'A':
        x_mod, y_mod, z_mod = 85,0,-10
    elif stage == 'B':
        x_mod, y_mod, z_mod = 9,0,0
    elif stage == 'C':
        x_mod, y_mod, z_mod = -67,0,-6


    colors = ["blue","blue","blue","blue","red","red","red","red","yellow","yellow","yellow","yellow","green","green","green","green"]
    coordinates = [(0,215.3,25),(0,215.3,28.81),(6.35,215.3,25),(6.35,215.3,28.81),(6.35*2,215.3,25),(6.35*2,215.3,28.81),(6.35*3,215.3,25),(6.35*3,215.3,28.81),(6.35*4,215.3,25),(6.35*4,215.3,28.81),(6.35*5,215.3,25),(6.35*5,215.3,28.81),(6.35*6,215.3,25),(6.35*6,215.3,28.81),(6.35*7,215.3,25),(6.35*7,215.3,28.81)]
    found_points = []
    for p in range(16):
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
                x=p[1][0]/100.0 + random.uniform(-.005, .005) + x_mod,
                y=p[1][1]/100.0 + random.uniform(-.005, .005) + y_mod,
                z=p[1][2]/100.0 + random.uniform(-.005, .005) + z_mod
            )
        )
    rospy.init_node('temp')
    point_pub = rospy.Publisher("/camera/block_point_cloud", PointCloud, queue_size=1)

    rgb_channels = [ChannelFloat32(name="r", values=channels[0]),
                    ChannelFloat32(name="g", values=channels[1]),
                    ChannelFloat32(name="b", values=channels[2])]

    print "Publishing..."
    point_pub.publish(PointCloud(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="map"
                ),
            points=points,
            channels=rgb_channels
        )
    )