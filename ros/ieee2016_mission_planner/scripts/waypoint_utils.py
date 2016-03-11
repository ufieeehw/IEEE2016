#!/usr/bin/env python
from __future__ import print_function

import rospkg
import getopt
import rospy
import sys
import tf
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospack = rospkg.RosPack()
WAYPOINTS_FILE_URI = os.path.join(rospack.get_path('ieee2016_mission_planner'), 'scripts/waypoints.txt')

def update_waypoints(old_names,new_names):
	# Used to reassign waypoint names. Inputs MUST be lists.
	waypoints_file = open(WAYPOINTS_FILE_URI,"r")
	old_waypoints = waypoints_file.read().split('\n')[:-1]

	waypoints_to_change = len(old_names)

	# Go through each of the old names and replace them in the new list with the new names
	for l in old_waypoints:
		new_waypoint = l
		for i in xrange(waypoints_to_change):
			if l.split(' ', 1)[0] == old_names[i]:
				save_waypoint_coor(new_names[i],l.split(' ', 1)[1].split())

def save_waypoint(name, x, y, theta):
		# Take a name and a coordinate [x y yaw] (with no []) and write them to waypoints file, updating if there are repeats.
	existing_waypoints = load_waypoints()
	coor = [x,y,theta]
	new_waypoint = {name:coor}

	existing_waypoints.update(new_waypoint)

	f = open(WAYPOINTS_FILE_URI,'w')
	for name in existing_waypoints:
		f.write("%s %s %s %s \n" % (name, x, y, theta))
	f.close()

def save_waypoint_coor(name, coor):
	# Take a name and a coordinate [x y yaw] (with no []) and write them to waypoints file, updating if there are repeats.
	existing_waypoints = load_waypoints()
	new_waypoint = {name:coor}

	existing_waypoints.update(new_waypoint)

	f = open(WAYPOINTS_FILE_URI,'w')
	for name in existing_waypoints:
		f.write("%s %s %s %s \n" % (name, existing_waypoints[name][0], existing_waypoints[name][1], existing_waypoints[name][2]))
	f.close()

def load_waypoints():
	# Load waypoints and return dictionary of points

	with open(WAYPOINTS_FILE_URI, "r") as in_file:
		lines = in_file.read().split()
		load_dict = {}
		#Puts all coordinates into a dictionary
		for i in range(len(lines) / 4):
			load_dict[lines[4*i]] = [float(lines[(4*i) + 1]), float(lines[(4*i) + 2]), float(lines[(4*i) + 3])]
		return load_dict	

def callback(data):
	pos = (data.pose.position.x, data.pose.position.y)
	#Converts quaternion coordinates to euler
	quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]

	save_waypoint(sys.argv[1], data.pose.position.x, data.pose.position.y, yaw)
	sub.unregister()
	print("Waypoint Saved:",sys.argv[1],data.pose.position.x,data.pose.position.y,yaw)
	rospy.signal_shutdown("Message saved")
	exit()

if __name__ == "__main__":
	global sub
	# when you call this script from the command line, it will save the current position as the first arguemnt of the call
	rospy.init_node('listener', anonymous=True)
	#print "listneing"
	sub = rospy.Subscriber("/robot/pf_pose_est", PoseStamped, callback)
	rospy.spin()
