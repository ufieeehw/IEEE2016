#!/usr/bin/env python
# Purpose of this script is to trim the laser's view to remove wheels and other obstructions from the edges
import rospy
import math
import sys

from sensor_msgs.msg import LaserScan

def trim_scan(scan,(pub,new_fov_angle)):
    # Find the new start and ending vertices coorispoding to the angular range
    s_index = len(scan.ranges)/2 - int(new_fov_angle/(2*scan.angle_increment))
    e_index = len(scan.ranges)/2 + int(new_fov_angle/(2*scan.angle_increment))

    if s_index < 0 or e_index > len(scan.ranges):
        raise Exception("New FOV bigger then old FOV")

    # The new scan is going to have same info (time and frame) except for the scan data and max/min
    trimmed_scan = scan
    trimmed_scan.ranges = scan.ranges[s_index:e_index]
    trimmed_scan.angle_min = -new_fov_angle/2
    trimmed_scan.angle_max = new_fov_angle/2

    rospy.loginfo("Publishing")
    pub.publish(trimmed_scan)

# User can pass new fov if they want (in degrees)
args = sys.argv
if len(sys.argv) > 1:
    new_fov_angle = math.radians(float(args[1]))
else:
    # 125 degree field of view if nothing is specified
    new_fov_angle = 2.18166


pub_left = rospy.Publisher('scan_trimmed_left', LaserScan, queue_size=5)
pub_middle = rospy.Publisher('scan_trimmed_middle', LaserScan, queue_size=5)
pub_right = rospy.Publisher('scan_trimmed_right', LaserScan, queue_size=5)

rospy.init_node('laser_trimmer')

rospy.loginfo(new_fov_angle)

rospy.Subscriber('scan_left',LaserScan,trim_scan,(pub_left,new_fov_angle))
rospy.Subscriber('scan_middle',LaserScan,trim_scan,(pub_middle,new_fov_angle))
rospy.Subscriber('scan_right',LaserScan,trim_scan,(pub_right,new_fov_angle))

rospy.loginfo("Ready for scan data...")

rospy.spin()
