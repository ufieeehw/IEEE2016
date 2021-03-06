#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
import cv2

def nothing(x):
    pass

# Create a black image, a window
params = np.zeros((1,1024,3), np.uint8)
cv2.namedWindow('params')

# The max values here should be changed to the max scan index
cv2.createTrackbar('min','params',0,512,nothing)
cv2.createTrackbar('max','params',0,512,nothing)

def average(ranges,reset):
    global average_list,not_fixed_list

    ranges = np.array(ranges)
    ranges = ranges[~np.isnan(ranges)]
    ranges = ranges[~np.isinf(ranges)]
    
    if len(ranges) <= 0:
        print "Invaild Range."
        return

    if reset:
        average_list = []
        not_fixed_list = []
        print "Reseting."
        
    avg = sum(ranges)/len(ranges)
    o_avg = np.copy(avg)
    not_fixed_list.append(avg)

    lidar_calibration = np.array([0.1081092694,-0.459223754,1.2548545404,-1.4490855292,0.7316648968,-0.1359287282])
    order = len(lidar_calibration)

    for c in range(order):
        print o_avg**c
        avg+=lidar_calibration[c] * o_avg**c
    print "Err:",o_avg-avg

    average_list.append(avg)

    print "Corrected:",avg#sum(average_list)/len(average_list)
    print "Raw:",o_avg#sum(not_fixed_list)/len(not_fixed_list)
    print

def callback(msg):
    global last_min, last_max, params, pub

    cv2.imshow('params',params)
    cv2.waitKey(1)
    min_index = cv2.getTrackbarPos('min','params')
    max_index = cv2.getTrackbarPos('max','params')

    trimmed_ranges = msg.ranges[min_index:max_index]
    reset = True
    if min_index == last_min and max_index == last_max:
        reset = False

    average(trimmed_ranges,reset)
    last_max = max_index
    last_min = min_index

    trimmed_scan = msg
    msg.angle_min = min_index*msg.angle_increment-3.1415/2.0
    msg.angle_max = max_index*msg.angle_increment-3.1415/2.0
    msg.ranges = trimmed_ranges
    pub.publish(trimmed_scan)

    #print msg.angle_max,msg.angle_min
	

rospy.init_node('lidar_distance')
rospy.Subscriber('/scan',LaserScan,callback)
pub = rospy.Publisher('scan_trim',LaserScan,queue_size=1)

vis_min = rospy.Subscriber
vis_max = rospy.Subscriber

r = rospy.Rate(10)
last_max = 0
last_min = 0
average_list = []
not_fixed_list = []
rospy.spin()
