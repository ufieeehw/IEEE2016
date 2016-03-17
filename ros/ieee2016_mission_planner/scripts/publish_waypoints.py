#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
from ieee2016_msgs.srv import NavWaypoint
import tf

from waypoint_utils import load_waypoints

publisher = rospy.Publisher('/nav_waypoint',PoseStamped,queue_size=2)
rospy.init_node('pose_publisher', anonymous=True)
nav_waypoint = rospy.ServiceProxy('/robot/nav_waypoint', NavWaypoint)

while not rospy.is_shutdown():
    print "=========================================================="
    waypoints = load_waypoints()
    for i in waypoints: print i 
    target_point = raw_input(" > Enter Target Waypoint: ")
    waypoints = load_waypoints()
    try:
        target = waypoints[target_point]


        q = tf.transformations.quaternion_from_euler(0, 0, target[2])
        p_s=PoseStamped(
            		header=Header(
                		stamp=rospy.Time.now(),
                		frame_id="map"
            		),
            		pose=Pose(
                			position=Point(
                    		x=target[0],
                    		y=target[1],
                    		z=0
                			),
                			orientation=Quaternion(
                    			x=q[0],
                    			y=q[1],
                    			z=q[2],
                    			w=q[3],
                			)
            		)
        	   	)
        publisher.publish(p_s)
        success = nav_waypoint(p_s)
        if success:
            print " > Movement Complete!"
        else:
            print " > Error, could not complete movement"
    except:
		print " > Not Found"

