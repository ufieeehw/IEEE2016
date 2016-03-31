#!/usr/bin/env python
#from __future__ import print_function

import rospkg
import getopt
import rospy
import sys
import tf
import os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospack = rospkg.RosPack()

class NavWaypointGenerator():
	'''
	Simple way of saving all the waypoints we need to run.

	Make sure all the vision waypoints are tested throughly.

	There's a chance that the pickup wapypoints for ee2 are 3.14 rads off of ee1 waypoints.

	Box_1 should be the leftmost box regardless of the map orientation.
	'''
	def __init__(self): 
		self.req_waypoints = ['through_box', 'vision_b_1', 'vision_b_2', 'vision_c_1', 'vision_c_2', 
		'pickup_a_ee1_block0','pickup_a_ee1_block1','pickup_a_ee1_block2','pickup_a_ee1_block3','pickup_a_ee1_block4','pickup_a_ee1_block5','pickup_a_ee1_block6','pickup_a_ee1_block7',
		'pickup_a_ee2_block0','pickup_a_ee2_block1','pickup_a_ee2_block2','pickup_a_ee2_block3','pickup_a_ee2_block4','pickup_a_ee2_block5','pickup_a_ee2_block6','pickup_a_ee2_block7',
		'pickup_b_ee1_block0','pickup_b_ee1_block1','pickup_b_ee1_block2','pickup_b_ee1_block3','pickup_b_ee1_block4','pickup_b_ee1_block5','pickup_b_ee1_block6','pickup_b_ee1_block7',
		'pickup_b_ee2_block0','pickup_b_ee2_block1','pickup_b_ee2_block2','pickup_b_ee2_block3','pickup_b_ee2_block4','pickup_b_ee2_block5','pickup_b_ee2_block6','pickup_b_ee2_block7',
		'pickup_c_ee1_block0','pickup_c_ee1_block1','pickup_c_ee1_block2','pickup_c_ee1_block3','pickup_c_ee1_block4','pickup_c_ee1_block5','pickup_c_ee1_block6','pickup_c_ee1_block7',
		'pickup_c_ee2_block0','pickup_c_ee2_block1','pickup_c_ee2_block2','pickup_c_ee2_block3','pickup_c_ee2_block4','pickup_c_ee2_block5','pickup_c_ee2_block6','pickup_c_ee2_block7',
		'safe_a_rotate', 'box_1','box_2','box_3','box_4', 'boat']

		map_version = raw_input("Which map version? ")

		rospy.Subscriber("/odometry/filtered", Odometry, self.got_odom)
		
		self.started = False
		self.pose = [0,0,0]
		
		self.w = WaypointServer(map_version)
		print "I'm just going to be waiting for the first odom message."

	def got_odom(self,msg):
		msg = msg.pose
		yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])[2]
		self.pose = np.array([msg.pose.position.x,msg.pose.position.y,yaw])
		if not self.started:
			print "Hey look! I just got the first message. Let's start calibrating shall we?"
			self.start_generating()

	def start_generating(self):
		self.started = True
		print
		for waypoint in self.req_waypoints:
			raw_input("Please move el robot to '%s'. (Press return to confirm)" % waypoint)
			self.w.save_waypoint_coor(waypoint, self.pose)
			print "Waypoint '%s' saved!"%waypoint
			print

		print "Good job! You finished. Goodluck out there buddy :)!"

class WaypointServer():
	def __init__(self, map_version = None):
		if map_version is None:
			map_request = rospy.ServiceProxy('/robot/request_map',RequestMap)
			map_version = np.array(map_request().map_version)
		self.waypoints_file_uri = os.path.join(rospack.get_path('ieee2016_mission_planner'), 'scripts/%s_waypoints.txt'%map_version)

	def switch_map_version(self, new_map_version):
		self.waypoints_file_uri = os.path.join(rospack.get_path('ieee2016_mission_planner'), 'scripts/%s_waypoints.txt'%new_map_version)

	def update_waypoints(self,old_names,new_names):
		# Used to reassign waypoint names. Inputs MUST be lists.
		waypoints_file = open(self.waypoints_file_uri,"r")
		old_waypoints = waypoints_file.read().split('\n')[:-1]

		waypoints_to_change = len(old_names)

		# Go through each of the old names and replace them in the new list with the new names
		for l in old_waypoints:
			new_waypoint = l
			for i in xrange(waypoints_to_change):
				if l.split(' ', 1)[0] == old_names[i]:
					save_waypoint_coor(new_names[i],l.split(' ', 1)[1].split())

	def save_waypoint(self, name, x, y, theta):
		#Take a name and a coordinate [x y yaw] (with no []) and write them to waypoints file, updating if there are repeats.
		existing_waypoints = self.load_waypoints()
		coor = [x,y,theta]
		new_waypoint = {name:coor}
		
		existing_waypoints.update(new_waypoint)

		f = open(self.waypoints_file_uri,'w')
		for name in existing_waypoints:
			f.write("%s %s %s %s \n" % (name, existing_waypoints[name][0], existing_waypoints[name][1], existing_waypoints[name][2]))
		f.close()

	def save_waypoint_coor(self, name, coor):
		# Take a name and a coordinate [x y yaw] (with no []) and write them to waypoints file, updating if there are repeats.
		existing_waypoints = self.load_waypoints()
		new_waypoint = {name:coor}

		existing_waypoints.update(new_waypoint)

		f = open(self.waypoints_file_uri,'w')
		for name in existing_waypoints:
			f.write("%s %s %s %s \n" % (name, existing_waypoints[name][0], existing_waypoints[name][1], existing_waypoints[name][2]))
		f.close()

	def load_waypoints(self):
		# Load waypoints and return dictionary of points
		with open(self.waypoints_file_uri, "r") as in_file:
			lines = in_file.read().split()
			load_dict = {}
			#Puts all coordinates into a dictionary
			for i in range(len(lines) / 4):
				load_dict[lines[4*i]] = [float(lines[(4*i) + 1]), float(lines[(4*i) + 2]), float(lines[(4*i) + 3])]
			return load_dict	

def callback(data):
	pose = data.pose.pose
	print(pose)
	pos = (pose.position.x, pose.position.y)
	#Converts quaternion coordinates to euler
	quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]

	w = WaypointServer()

	w.save_waypoint(sys.argv[1], pose.position.x, pose.position.y, yaw)
	sub.unregister()
	print "Waypoint Saved:",sys.argv[1],pose.position.x, pose.position.y,yaw	
	rospy.signal_shutdown("Message saved")
	exit()

if __name__ == "__main__":
	global sub
	# when you call this script from the command line, it will save the current position as the first arguemnt of the call
	rospy.init_node('waypoint_utils', anonymous=True)
	if len(sys.argv) > 1:
		sub = rospy.Subscriber("/odometry/filtered", Odometry, callback)
	else:
		w = NavWaypointGenerator()

	rospy.spin()
