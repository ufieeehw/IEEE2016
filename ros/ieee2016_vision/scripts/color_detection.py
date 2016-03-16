#!/usr/bin/python2
#=============================================================================
# Project: IEEE 2016 Hardware Team Robot (Shia LaBot)
# Module: Color Detection												v3.1
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import camera_manager
from color_calibration import Calibrations
import cv2
import numpy as np
import point_intersector
import rospy
import waypoint_utils


class Image():
	'''
	Contains various custom image manipulation methods. Some methods require
	the selection of a color to proceed. Colors id's are as follows: red,
	green, blue, and yellow.
	'''
	def __init__(self, camera, calibration, width = 640):
		self.camera = camera
		self.calibration = calibration
		self.frame = self.camera.image

		# Store original and operating image dimensions
		self.original_dimensions = (self.frame.shape[1], self.frame.shape[0])
		aspect_ratio = float(self.original_dimensions[0]) / self.original_dimensions[1]
		self.operating_dimensions = (width, int(width / aspect_ratio))
		self.scaling_ratio = float(self.original_dimensions[0]) / self.operating_dimensions[0]

		self.hold_redux_frame = False

	def resize(self):
		'''
		Resizes a frame from the video feed to the operating image with and
		proportional height.
		'''
		self.frame = self.camera.image
		self.frame = cv2.resize(self.frame, self.operating_dimensions, interpolation = cv2.INTER_AREA)

	def reformat_to_rgb(self):
		self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

	def scale_point(self, point):
		'''
		Scales a point from an image of lower resolution to the camera's true
		image resolution.
		'''
		for i in range(2):
			point[i] = point[i] * self.scaling_ratio

		return point

	def reduce_colors(self, image_color_depth):
		'''
		Reduces the range of colors present in a frame of the video feed based on
		the set color depth.
		'''
		# Can be set to reuse the reduced color frame to reduce processing cost
		if (self.hold_redux_frame):
			self.frame = self.redux_frame

		else:
			self.resize()

			# Converts the frame to a format that is usable with K-Means clustering
			working_frame = self.frame.reshape((-1, 3))
			working_frame = np.float32(working_frame)

			# Use K-Means clustering to identify the 16 most predominant colors
			criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
			ret, label, center = cv2.kmeans(working_frame, image_color_depth, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

			# Reduces the colors in the original image based on the clustering results
			center = np.uint8(center)
			working_frame = center[label.flatten()]
			working_frame = working_frame.reshape((self.frame.shape))

		 	# Filters the remaining colors to reduce noise
 		 	working_frame = cv2.medianBlur(working_frame, 5)
 		 	self.frame = cv2.bilateralFilter(working_frame, 9, 75, 75)

		 	# Stores the frame for later holding
		 	self.redux_frame = self.frame

	def extract_color(self, color):
		'''
		Generates an image from the passed frame that contain hues specified within
		the defined boundaries for the color id passed.
		'''
		self.reduce_colors(32)

		# Creates a mask for the selected hue range and overlays them onto the frame
		working_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(working_frame, self.calibration.colors[color][0], self.calibration.colors[color][1])
		self.frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)


class ObjectDetection():
	'''
	Generates usable data about the location of objects within the frame.
	Colors id's are as follows: red, green, blue, and yellow.
	'''
	def __init__(self, camera, calibration, image):
		self.image = image

		self.boxes = {}
		self.box_centers = {}

	def select_largest_object(self, colors):
		'''
		Selects the largest object in a frame by finding all contours and
		selecting the largest one.
		'''
		for color in colors:
			largest_area = None
			largest_contour = None

			# Reuse the reduced color frame for all colors to reduce processing cost
			self.image.extract_color(color)
			self.image.hold_redux_frame = True

			# Generates a gray frame for the passed color id
	 		working_frame = cv2.cvtColor(self.image.frame, cv2.COLOR_BGR2GRAY)

			# Finds all contours within the frame
			ret, thresh = cv2.threshold(working_frame, 50, 255, 0)
			contours, hierarchy = cv2.findContours(thresh, 1, 2)

			# Selects the largest contour in the frame
			for contour in contours:
				area = cv2.contourArea(contour)
				if (area > largest_area):
					largest_area = area
					largest_contour = contour

			if (largest_contour != None):
				rect = cv2.minAreaRect(largest_contour)
				self.boxes[color] = cv2.cv.BoxPoints(rect)
			else:
				self.boxes[color] = None

		# Release the reduced color frame that is being held
		self.image.hold_redux_frame = False

	def average_box(self, function, colors, amount):
		'''
		Defines a new set of box points that is the result of averaging
		'amount' number of box points from the chosen method (this method
		should set self.box). Will retry a limited number of times before
		simply setting self.box to None.
		'''
		max_retry = amount / 4
		retry = {}
		box_avg = {}

		for color in colors:
			retry[color] = 1
			box_avg[color] = [[0, 0], [0, 0], [0, 0], [0, 0]]

		for run in range(amount):
			# Run the passed function (should be one from this class object)
			function(colors)

			for color in colors:
				# Add up all of the box points' (x, y) values
				if (self.boxes[color]):
					for point in range(4):
						for value in range(2):
							box_avg[color][point][value] = box_avg[color][point][value] + self.boxes[color][point][value]

				# Aborts the averaging if a set number of measurements return None
				elif (retry[color] == max_retry):
					self.boxes[color] = None
					break

				# Attempts to retry failed measurements
				else:
					retry[color] += 1
					run -= 1

		for color in colors:
			# Divides the sums of the points' (x, y) values by the amount of values
			if (self.boxes[color]):
				for point in range(4):
					for value in range(2):
						box_avg[color][point][value] = box_avg[color][point][value] / amount
				self.boxes[color] = box_avg[color]

	def get_box_center(self):
		'''
		Finds the center point of the class object's box variable by averaging
		the (x, y) points of the corners. If there is no box, it sets self.box
		to None.
		'''
		box_avg = {}

		for color in self.boxes.keys():
			box_avg[color] = [0, 0]

			if (self.boxes[color]):
				for point in range(4):
					for value in range(2):
						box_avg[color][value] = box_avg[color][value] + self.boxes[color][point][value]
				for value in range(2):
					box_avg[color][value] = (int)(box_avg[color][value] / len(self.boxes[color]))
				self.box_centers[color] = tuple(box_avg[color])
			else:
				self.box_centers[color] = None

	def draw_box(self, colors, box_color):
		'''
		Draw the box based on the box points around the selected object onto
		the frame.
		'''
		for color in colors:
			if (self.boxes[color]):
				box_to_draw = np.int0(self.boxes[color])
				cv2.drawContours(self.image.frame, [box_to_draw], 0, box_color, 2)

	def draw_center_point(self, colors, point_color):
		'''
		Draw the center point of the box onto the frame.
		'''
		for color in colors:
			if (self.boxes[color]):
				cv2.circle(self.image.frame, self.box_centers[color], 2, point_color, 4)


class TrainBoxes():
	'''
	Used to locate and determine the order of the colored train boxes at
	the beginning of a run. Makes use of existing waypoints in order to
	infer the location of any box that was not properly detected.
	'''
	def __init__(self, camera):
		self.camera = camera
		self.calibration = Calibrations()
		self.image = Image(camera, calibration, 320)
		self.detection = ObjectDetection(self.camera, self.calibration)
		self.waypoints = waypoint_utils.load_waypoints()
		self.intersect = point_intersector.PointIntersector()

	def get_box_location(self, averaging):
		'''
		Determines the locations of the boxes in Shia's coordinate system based
		on their location within the given camera frame.
		'''
		self.box_locations = {}
		self.not_detected = []

		self.camera.activate()
		self.detection.average_box(self.detection.select_largest_object, self.colors , averaging)
		self.detection.get_box_center()
		self.camera.deactivate()
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


def debug_selection(camera, colors, averaging):
	'''
	Used to debug the selection of objects based on hue and relative size in
	the frame. Displays real-time selection output using cv2 for feedback.
	'''
	camera.activate()
	calibration = Calibrations()
	image = Image(camera, calibration, 320)
	detection = ObjectDetection(camera, calibration, image)

	while (True):
		# Finds the center of the box around the largest object
		detection.average_box(detection.select_largest_object, colors, averaging)
		detection.get_box_center()

		# Pulling values used to render the debugging image
		image.resize()
		frame = image.frame
		box = detection.boxes
		box_centers = detection.box_centers

		for color in colors:
			# Draw a box around the selected object in the captured frame
			if (box[color]):
				box_to_draw = np.int0(box[color])
				cv2.drawContours(frame, [box_to_draw], 0, (0, 0, 255), 2)

			# Draw the center point of the box in the captured frame
			if (box_centers[color]):
				cv2.circle(frame, box_centers[color], 3, (0, 255, 0))

		# Display the frame for debugging
		cv2.imshow('Debugging', frame)
		if (cv2.waitKey(5) == 27):
			cv2.destroyAllWindows
			break

	camera.deactivate()

if __name__ == "__main__":
	rospy.init_node("color_dection")
	camera = camera_manager.Camera(1)
	debug_selection(camera, ["red"], 2)
	# train = TrainBoxes(camera)
	# train.get_box_order(["red", "green", "blue", "yellow"], ["box_1", "box_2", "box_3", "box_4"], 8)
	exit()
