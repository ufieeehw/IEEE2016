#!/usr/bin/python2
#==============================================================================
# Project: Machine Vision - Color Detection
# Module: Boxes															v2.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import os
import rospkg

from calibrator import CalibrationFile, Image, ObjectDetector
from camera_manager import Camera
import point_intersector
import rospy
import waypoint_utils


class TrainBoxes():
	'''
	Used to locate and determine the order of the colored train boxes at
	the beginning of a run. Makes use of existing waypoints in order to
	infer the location of any box that was not properly detected.
	'''
	def __init__(self, camera, waypoint_server):
		# Imports the default calibration file from the colors directory
		rospack = rospkg.RosPack()
		self.__file_path = os.path.join(rospack.get_path("ieee2016_vision"), "scripts/calibrations/color_calibrations.yaml")
		self.__calibration_file = CalibrationFile(self.__file_path, "load", [320, 180])

		# Objects used in the process of detection
		self.__camera = camera
		self.__image = Image(self.__camera, self.__calibration_file, 320)
		self.__detector = ObjectDetector(self.__image)
		self.__waypoints = waypoint_server.load_waypoints()
		self.__intersector = point_intersector.PointIntersector()

	def __calculate_box_location(self, averaging):
		'''
		Determines the locations of the boxes in Shia's coordinate system based
		on their location within the given camera frame.
		'''
		self.__box_locations = {}
		self.__not_detected = []

		self.__detector.average_box(self.__detector.select_largest_solid, self.__colors , averaging)
		for color in self.__colors:
			center = self.__detector.get_box_center(color)

			# Uses intersect_point to determine the 3D location of each detected box center
			if (center):
				self.__box_locations[color] = self.__intersector.intersect_point(self.__camera, self.__image.scale_point(center))

			# If detection was unsuccessful for a color, adds it to not_detected and prints an error
			else:
				self.__not_detected.append(color)
				print "Warning the detector could not detect the '%s' box" % (color)

	def __infer_box_location(self):
		'''
		Determines the location of a single box that was not properly detected
		using the positions of the other boxes. Can only be used once all of
		the other boxes have been assigned colors!
		'''
		if ((len(self.__not_detected) == 1) and (not len(self.__matched_boxes) < len(self.__locations))):
			for location in self.__locations:
				if (not location in self.__matched_boxes.keys()):
					self.__matched_boxes[location] = self.__not_detected.pop()

	def __waypoint_approximation(self):
		'''
		Determines the waypoints that best match the 3D points obtained
		through get_box_locations.
		'''
		for location in self.__locations:

			# An initialization value larger than any possible distance on the field
			plumbusi_constant = 42069133742
			smallest_distance = plumbusi_constant
			closest_color = None

			# Assigns the color closest to the location to that location
			for color in self.__colors:

				# Will not be run for colors that were not detected
				if (color in self.__box_locations):
					distance = abs(self.__box_locations[color][1] - self.__waypoints[location][1])

					# Determines which color is closest to each waypoint
					if (distance < smallest_distance):
						smallest_distance = distance
						closest_color = color
			self.__matched_boxes[location] = closest_color

		# Removes any colors that were assigned to more than one location
		cleaned_matching = {}
		for key, value in self.__matched_boxes.iteritems():
			if (value in cleaned_matching.iterkeys()):
				del cleaned_matching[value]
			else:
				cleaned_matching[value] = key
		self.__matched_boxes = dict((value, key) for key, value in cleaned_matching.iteritems())

	def get_box_order(self, colors, locations, averaging):
		'''
		Returns the formated order of the boxes. They will be in a dictionary
		with the keys being waypoint names and the values being colors.
		'''
		# The colors of boxes to locate
		self.__colors = colors

		# The waypoints of the boxes
		self.__locations = locations

		# Amount of frames to average - increases reliability, recommended minimum of 8
		self.__averaging = averaging

		# The table to be returned keys are waypoint names and values are colors
		self.__matched_boxes = {}

		self.__calculate_box_location(averaging)
		self.__waypoint_approximation()
		self.__infer_box_location()

		# Sets all of the boxes that were not detected to None
		for location in self.__locations:
			if (not location in self.__matched_boxes.keys()):
				self.__matched_boxes[location] = None

		return self.__matched_boxes

if __name__ == "__main__":
	rospy.init_node("boxes")
	camera = Camera(1)
	camera.activate()
	waypoint_server = waypoint_utils.WaypointServer(1)
	train = TrainBoxes(camera, waypoint_server)
	print train.get_box_order(["red", "blue", "yellow"], ["box_1", "box_2", "box_3"], 8)
	camera.deactivate()
	exit()
