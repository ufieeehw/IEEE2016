#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
import tf

from waypoint_utils import load_waypoints

publisher = rospy.Publisher('/robot/waypoint',PoseStamped,queue_size=2)
rospy.init_node('pose_publisher', anonymous=True)


while not rospy.is_shutdown():

	waypoints = load_waypoints()
	print waypoints
	target_point = raw_input("Enter Target Waypoint: ")
	waypoints = load_waypoints()
	try:
		target = waypoints[target_point]
		print target

        	q = tf.transformations.quaternion_from_euler(0, 0, target[2])
        	publisher.publish(
            		PoseStamped(
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
        	)

	except:
		print "Not Found"

