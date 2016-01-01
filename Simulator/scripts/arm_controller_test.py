#!/usr/bin/env python
import tf
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import geometry_msgs.msg
import math
import numpy as np

#THIS IS MEANT FOR THE SIMULATOR ONLY!
#THE COMMANDS SENT TO THE CONTROLLER ARE NOT REALIZABLE AND WILL BREAK THINGS!
'''This publisher sends out PointStamped commands for the arm to spin in a circle. 
   This is a tool to used for assessing the simulator.
   USAGE:
	Set r to the desired radius
	rosrun ieee2016_simulator arm_controller_test
'''
def point(xcomp, ycomp, r):
    dist1 = (r*xcomp/10, r*ycomp/10, 0.0)
    dist2 = Point(*dist1)
    dist = PointStamped()               
    dist.header.stamp = rospy.Time.now()
    dist.header.frame_id = '/robot'
    dist.point = dist2
    des_pose.publish(dist)
    return dist

if __name__== "__main__":
    rospy.init_node('arm_controller_test')
    while(True):
        time = rospy.get_time()
        des_pose = rospy.Publisher('/sim/arm_des_pose', PointStamped, queue_size=10)
        try:
            x=np.cos(time)
            y=np.sin(time)
            r=3.05
            point(x,y, r)
        except rospy.ROSInterruptException:
            pass
        rospy.sleep(0.2)

    
