#!/usr/bin/env python

import getopt
import rospy
import sys
import tf
import os
from nav_msgs.msg import Odometry

def save_waypoint(name, x, y, theta):
	x_cor = str(x)
	y_cor = str(y)
	angle = str(theta)

	f = os.path.join(os.path.dirname(__file__), 'waypoints.txt')

	with open(f, "r") as in_file:
		#Creates and writes all waypoints into a file before doing analysis
		out_file = open(f, "a+")
		out_file.write("%s %s %s %s \n" % (name, x_cor, y_cor, angle))

		#Creates a big list of all with each element as a separate index
		lines = in_file.read().split()
		list_len = len(lines)

		#Makes a list of all unique names
		names = []
		for x in range(list_len / 4):
			names.append(lines[4 * x])

		#Variables to be used for the unique waypoints
		new_file = open(f, "r")
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
	f = os.path.join(os.path.dirname(__file__), 'waypoints.txt')

	with open(f, "r") as in_file:
		lines = in_file.read().split()
		load_dict = {}
		#Puts all coordinates into a dictionary
		for i in range(len(lines) / 4):
			load_dict[lines[4*i]] = [float(lines[(4*i) + 1]), float(lines[(4*i) + 2]), float(lines[(4*i) + 3])]
		return load_dict	

def callback(data):
	#Converts quaternion coordinates to euler
	quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]

	save_waypoint(sys.argv[1], roll, pitch, yaw)
	sub.unregister()
	rospy.signal_shutdown("Message saved")

def listener():
	rospy.init_node('listener', anonymous=True)
	global sub
	sub = rospy.Subscriber("robot_odom_filter", Odometry, callback)
	rospy.spin()

if __name__ == "__main__":
	listener()