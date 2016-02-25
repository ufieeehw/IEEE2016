#!/usr/bin/env python
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

	# Close then reopen the file for writing
	waypoints_file.close()
	waypoints_file = open(WAYPOINTS_FILE_URI,"w")

	waypoints_to_change = len(old_names)

	# Go through each of the old names and replace them in the new list with the new names
	for l in old_waypoints:
		new_waypoint = l
		for i in xrange(waypoints_to_change):
			if l.split(' ', 1)[0] == old_names[i]: 
				new_waypoint = str(new_names[i] + " " + l.split(' ', 1)[1])

  		waypoints_file.write("%s\n" % new_waypoint)

def save_waypoint(name, x, y, theta):
	# Save waypoint with 'name' at [x,y,theta]
	x_cor = str(x)
	y_cor = str(y)
	angle = str(theta)

	with open(WAYPOINTS_FILE_URI, "r") as in_file:
		#Creates and writes all waypoints into a file before doing analysis
		out_file = open(WAYPOINTS_FILE_URI, "a+")
		out_file.write("%s %s %s %s \n" % (name, x_cor, y_cor, angle))

		#Creates a big list of all with each element as a separate index
		lines = in_file.read().split()
		list_len = len(lines)

		#Makes a list of all unique names
		names = []
		for x in range(list_len / 4):
			names.append(lines[4 * x])

		#Variables to be used for the unique waypoints
		new_file = open(WAYPOINTS_FILE_URI, "r")
		update_lines = new_file.readlines()

		for x in range(0, (list_len / 4) - 1):
			#Loops through the list of names and finds if there is a match
			for y in range(list_len / 4):

				#If a duplicate is found, deletes the oldest coordinate with the matching name and rewrites the file
				if name == names[y]:
					dupe = names.index(name)
					del update_lines[dupe]

					file_update = open(f, "w")
					file_update.writelines(update_lines)	

					file_update.close()
					return
										
		in_file.close()
		new_file.close()

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
	print "Waypoint Saved:", sys.argv[1],data.pose.position.x,data.pose.position.y,yaw
	rospy.signal_shutdown("Message saved")
	exit()

if __name__ == "__main__":
	global sub
	# when you call this script from the command line, it will save the current position as the first arguemnt of the call
	rospy.init_node('listener', anonymous=True)
	#print "listneing"
	sub = rospy.Subscriber("/robot/pf_pose_est", PoseStamped, callback)
	rospy.spin()
