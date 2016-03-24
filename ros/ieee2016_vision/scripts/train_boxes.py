#!/usr/bin/python2
#=============================================================================
# Project: Machine Vision - Color Detection
# Module: Train Boxes												v1.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import camera_manager
from color_calibration import CalibrationData
import point_intersector
import rospy
import waypoint_utils


class TrainBoxes():
	'''
	Used to locate and determine the order of the colored train boxes at
	the beginning of a run. Makes use of existing waypoints in order to
	infer the location of any box that was not properly detected.
	'''
	def __init__(self, camera):
		# Imports the default calibration file from the colors directory
		rospack = rospkg.RosPack()
		self.calibration_file = os.path.join(rospack.get_path("ieee2016_vision"), "scripts/color/color_calibrations.json")
		self.calibration_file = CalibrationData()

		# Objects used in the process of detection
		self.camera = camera
		self.image = Image(camera, calibration, 320)
		self.detection = ObjectDetection(self.camera, self.calibration_file)
		self.waypoints = waypoint_utils.load_waypoints()
		self.intersect = point_intersector.PointIntersector()

	def get_box_location(self, averaging):
		'''
		Determines the locations of the boxes in Shia's coordinate system based
		on their location within the given camera frame.
		'''
		self.box_locations = {}
		self.not_detected = []

		self.detection.average_box(self.detection.select_largest_solid, self.colors , averaging)
		self.detection.get_box_center()
		for color in self.colors:

			# Uses intersect_point to determine the 3D location of each detected box center
			if (self.detection.box_centers[color]):
				self.box_locations[color] = self.intersect.intersect_point(self.camera, self.image.scale_point(self.detection.box_centers[color]))

			# If detection was unsuccessful for a color, adds it to not_detected and prints an error
			else:
				self.not_detected.append(color)
				print("WARNING: color_detection could not detect the center of the %s box" % (color))

	def infer_box_location(self):
		'''
		Determines the location of a single box that was not properly detected
		using the positions of the other boxes. Can only be used once all of
		the other boxes have been assigned colors!
		'''
		if (len(self.not_detected) == 1):
			for location in self.locations:
				if (not location in self.matched_boxes.keys()):
					self.matched_boxes[location] = self.not_detected.pop()

	def waypoint_approximation(self):
		'''
		Determines the waypoints that best match the 3D points obtained
		through get_box_locations.
		'''
		for location in self.locations:

			# Ensures that two colors are not assigned to a box
			if (not location in self.matched_boxes.keys()):

				# An initialization value larger than any possible distance on the field
				smallest_distance = 42069133742
				closest_color = None

				# Assigns the color closest to the location to that location
				for color in self.colors:
					distance = abs(self.box_locations[color][1] - self.waypoints[color][1])
					if (distance < smallest_distance):
						smallest_distance = distance
						closest_color = color
				self.matched_boxes[location] = color

			# Catch all for a situation in which no color is closest to the location
			else:
				self.matched_boxes[location] = None

		# Removes any colors that were assigned to more than one location
		cleaned_matching = {}
		for key, value in self.matched_boxes.items():
			if value not in cleaned_matching.values():
				cleaned_matching[key] = value
		self.matched_boxes = cleaned_matching

	def get_box_order(self, colors, locations, averaging):
		'''
		Returns the formated order of the boxes. They will be in a dictionary
		with the keys being waypoint names and the values being colors.
		'''
		# The colors of boxes to locate
		self.colors = colors

		# The waypoints of the boxes
		self.locations = locations

		# Amount of frames to average - increases reliability, recommended minimum of 8
		self.averaging = averaging

		# The table to be returned keys are waypoint names and values are colors
		self.matched_boxes = {}

		self.get_box_location(averaging)
		self.waypoint_approximation()
		self.infer_box_location()

		# Sets all of the boxes that were not detected to None
		for location in self.locations:
			if (not location in self.matched_boxes.keys()):
				self.matched_boxes[location] = None

		return(self.matched_boxes)

if __name__ == "__main__":
	rospy.init_node("train_boxes")
	camera = camera_manager.Camera(1)
	train = TrainBoxes(camera)
	train.get_box_order(["red", "green", "blue", "yellow"], ["box_1", "box_2", "box_3", "box_4"], 8)
	exit()
