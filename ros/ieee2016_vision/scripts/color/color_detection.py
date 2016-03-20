#!/usr/bin/python2
#=============================================================================
# Project: Machine Vision - Color Detection
# Module: Color Detection												v4.1
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

from color_calibration import CalibrationData
import cv2
import numpy as np


class Image():
	'''
	Contains various custom image manipulation methods. Some methods require
	the selection of a color to proceed. Colors id's are as follows: red,
	green, blue, and yellow.
	'''
	def __init__(self, camera, calibration_file, width = 640):
		self.camera = camera
		self.calibration_file = calibration_file
		self.frame = self.camera.image

		# Store original and operating image dimensions and associated ratios
		self.original_dimensions = (self.frame.shape[1], self.frame.shape[0])
		self.aspect_ratio = float(self.original_dimensions[0]) / self.original_dimensions[1]
		self.operating_dimensions = (width, int(width / self.aspect_ratio))
		self.scaling_ratio = float(self.original_dimensions[0]) / self.operating_dimensions[0]

		self.hold_redux_frame = False

	def resize(self, width = None):
		'''
		Resizes a frame from the video feed to the operating image with and
		proportional height. If a width is specified, a frame scaled to that
		width by the aspect ratio is returned.
		'''
		self.frame = self.camera.image

		if (width):
			width = (width / 4) * 4
			height = int(width / self.aspect_ratio)
			new_dimensions = (width, height)
		else:
			new_dimensions = self.operating_dimensions

		self.frame = cv2.resize(self.frame, new_dimensions, interpolation = cv2.INTER_AREA)

	def reformat_to_rgb(self):
		self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

	def scale_point(self, point):
		'''
		Scales a point from an image of lower resolution to the camera's true
		image resolution. The scaling ratio is based on the operating image
		dimensions.
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

		 	# Filters the image to reduce noise
 		 	working_frame = cv2.bilateralFilter(self.frame, 9, 75, 75)
 		 	working_frame = cv2.medianBlur(working_frame, 5)

			# Converts the frame to a format that is usable with K-Means clustering
			working_frame = working_frame.reshape((-1, 3))
			working_frame = np.float32(working_frame)

			# Use K-Means clustering to identify the 16 most predominant colors
			criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
			ret, label, center = cv2.kmeans(working_frame, image_color_depth, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

			# Reduces the colors in the original image based on the clustering results
			center = np.uint8(center)
			working_frame = center[label.flatten()]
			self.frame = working_frame.reshape((self.frame.shape))

		 	# Stores the frame for later holding
		 	self.redux_frame = self.frame

	def extract_color(self, color):
		'''
		Generates an image from the passed frame that contain hues specified within
		the defined boundaries for the color id passed.
		'''
		self.reduce_colors(16)

		# Creates a mask for the selected hue range and overlays them onto the frame
		working_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(working_frame, self.calibration_file.colors[color][0], self.calibration_file.colors[color][1])
		self.frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)


class ObjectDetection():
	'''
	Generates usable data about the location of objects within the frame.
	Colors id's are as follows: red, green, blue, and yellow.
	'''
	def __init__(self, camera, image):
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
