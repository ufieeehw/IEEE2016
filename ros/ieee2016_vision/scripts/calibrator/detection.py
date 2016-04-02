#!/usr/bin/python2
#==============================================================================
# Project: Machine Vision - Color Detection
# Module: Detection														v5.1
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

from sklearn.cluster import KMeans

import cv2
import numpy as np


class Image():
	'''
	Contains various custom image manipulation methods. Some methods require
	the selection of a color to proceed. Colors id's are as follows: red,
	green, blue, and yellow.
	'''
	def __init__(self, camera, calibration_file, width = 480):
		self.__camera = camera
		self.__calibration_file = calibration_file
		self.__frame = self.__camera.get_frame()

		# Store original and operating image dimensions and associated ratios
		self.__original_dimensions = (self.__frame.shape[1], self.__frame.shape[0])
		self.__aspect_ratio = float(self.__original_dimensions[0]) / self.__original_dimensions[1]
		self.__operating_dimensions = (width, int(width / self.__aspect_ratio))
		self.__scaling_ratio = float(self.__original_dimensions[0]) / self.__operating_dimensions[0]

		# Enables reuse of the reduced color frame
		self.__hold_reduced = False
		self.__reduced_frame = {}

	def get_frame(self):
		'''
		Returns the frame that is currently stored in the cache.
		'''
		return self.__frame

	def reformat_to_rgb(self):
		'''
		Converts the frame from BGR to RGB for a frame display and returns it.
		'''
		return cv2.cvtColor(self.__frame, cv2.COLOR_BGR2RGB)

	def set_hold_reduced(self, state):
		'''
		Sets the state of the hold_reduced variable. This is used to hold a
		reduced color frame when it is not imperative to generate a new one
		in order to reduce processing cost.
		'''
		if (not type(state) == bool):
			raise TypeError("The value of hold_reduced must be a boolean")

		else:
			self.__hold_reduced = state

	def scale_point(self, point):
		'''
		Scales a point from an image of lower resolution to the camera's true
		image resolution. The scaling ratio is based on the operating image
		dimensions.
		'''
		if (len(point) != 2):
			raise TypeError("The value that was passed is not an (x, y) point")

		elif ((point[0] > self.__original_dimensions[0]) or (point[1] > self.__original_dimensions[1])):
			print("Warning: The point that was passed is not within the boundaries of the frame")

		else:
			for i in range(2):
				point[i] = point[i] * self.__scaling_ratio

			return point

	def resize(self):
		'''
		Resizes a frame from the video feed to the operating image with and
		proportional height.
		'''
		working_frame = self.__camera.get_frame()
		self.__frame = cv2.resize(working_frame, self.__operating_dimensions, interpolation = cv2.INTER_AREA)

	def __resize_to(self, width):
		'''
		Resizes the EXISTING frame in this object to the specified width and
		proportional height. Used to improve K-Means performance by reducing
		the amount of pixles to calculate for.
		'''
		width = (width / 4) * 4
		height = int(width / self.__aspect_ratio)
		new_dimensions = (width, height)
		self.__frame = cv2.resize(self.__frame, new_dimensions, interpolation = cv2.INTER_AREA)

	def draw_point(self, point):
		'''
		Draw a point onto the frame as a diamond.
		'''
		if (len(point) != 2):
			raise TypeError("The value that was passed is not an (x, y) point")

		elif ((point[0] > self.__original_dimensions[0]) or (point[1] > self.__original_dimensions[1])):
			raise ValueError("The point that was passed is not within the boundaries of the frame")

		else:
			cv2.circle(self.__frame, tuple(point), 2, (255, 255, 255), 4)

	def draw_box(self, box_points):
		'''
		Draw the box based on the specified box points onto the frame.
		'''
		if (len(box_points) != 4):
			raise TypeError("The value of box_points must be four (x, y) points")

		else:
			for point in box_points:
				if (len(point) != 2):
					raise TypeError("The value of box_points must be four (x, y) points")
				elif ((point[0] > self.__original_dimensions[0]) or (point[1] > self.__original_dimensions[1])):
					raise ValueError("One or more of the specified points was not within the boundaries of the frame")

			box_to_draw = np.int0(box_points)
			cv2.drawContours(self.__frame, [box_to_draw], 0, (255, 255, 255), 2)

	def reduce_colors(self, image_color_depth):
		'''
		Reduces the range of colors present in a frame of the video feed based on
		the set color depth.
		'''
		if (image_color_depth > 32):
			print("Warning: Setting image_color_depth to %d will severely impact performance" % (image_color_depth))

		# Can be set to reuse the reduced color frame to reduce processing cost
		elif ((self.__hold_reduced) and (image_color_depth in self.__reduced_frame.keys())):
			self.__frame = self.__reduced_frame[image_color_depth]

		else:
		 	# Filters a new frame to reduce noise
			self.resize()
 		 	working_frame = cv2.bilateralFilter(self.__frame, 20, 75, 75)

			# Downsamples the frame and converts it to a 2D floating point array
			self.__resize_to(32)
			working_frame = np.float32(self.__frame.reshape((-1, 3)))

			# Uses K-Means clustering to determine the requested highest density colors
			kmeans = KMeans(image_color_depth, "k-means++", 10, 4, 1e-4, "auto", 0, None, False, 1)
			kmeans.fit(working_frame)

		 	# Filters the image to reduce noise
			self.resize()
 		 	working_frame = cv2.bilateralFilter(self.__frame, 9, 75, 75)

			# Downsamples a new frame and converts it to a 2D floating point array
 		 	self.__resize_to(int(self.__operating_dimensions[0] / 6.5) * 4 / 4)
			working_frame = np.float32(self.__frame.reshape((-1, 3)))

			# Reduces the colors in the frame based on the clustering results
			labels = kmeans.fit_predict(working_frame)
			center = np.uint8(kmeans.cluster_centers_)
			working_frame = center[labels]

			# Resizes the frame to the operating parameters
			self.__frame = working_frame.reshape((self.__frame.shape))
			self.__resize_to(self.__operating_dimensions[0])

			# Stores the frame for later holding
			self.__reduced_frame[image_color_depth] = self.__frame

	def extract_color(self, color):
		'''
		Generates an image from the passed frame that contain hues specified within
		the defined boundaries for the color id passed.
		'''
		self.reduce_colors(16)

		# Creates a mask for the selected hue range and overlays them onto the frame
		working_frame = cv2.cvtColor(self.__frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(working_frame, self.__calibration_file.get_np_hsv_range(color)[0], self.__calibration_file.get_np_hsv_range(color)[1])
		self.__frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)


class ObjectDetector():
	'''
	Generates usable data about the location of objects within the frame.
	Colors id's are as follows: red, green, blue, and yellow.
	'''
	def __init__(self, image):
		self.__image = image
		self.__boxes = {}

	def average_box(self, function, colors, amount):
		'''
		Defines a new set of box points that is the result of averaging
		'amount' number of box points from the chosen method (this method
		should set self.box). Will retry a limited number of times before
		simply setting self.box to None.
		'''
		if (type(colors) != list):
			raise TypeError("Colors must be passed in a list")

		if (amount > 16):
			print("Warning: Averaging %d frames will severely impact performance" % (amount))

		else:
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
					if (self.__boxes[color]):
						for point in range(4):
							for value in range(2):
								box_avg[color][point][value] = box_avg[color][point][value] + self.__boxes[color][point][value]

					# Aborts the averaging if a set number of measurements return None
					elif (retry[color] == max_retry):
						self.__boxes[color] = None
						break

					# Attempts to retry failed measurements
					else:
						retry[color] += 1
						run -= 1

			for color in colors:
				# Divides the sums of the points' (x, y) values by the amount of values
				if (self.__boxes[color]):
					for point in range(4):
						for value in range(2):
							box_avg[color][point][value] = box_avg[color][point][value] / amount
					self.__boxes[color] = box_avg[color]

	def select_largest_solid(self, colors):
		'''
		Selects the largest object in a frame by finding all contours and
		selecting the largest one.
		'''
		if (type(colors) != list):
			raise TypeError("Colors must be passed in a list")

		else:
			for color in colors:

				# Reuse the reduced color frame for all colors to reduce processing cost
				self.__image.extract_color(color)
				self.__image.set_hold_reduced(True)
		 		working_frame = cv2.cvtColor(self.__image.get_frame(), cv2.COLOR_BGR2GRAY)

				# Finds all contours within the frame
				ret, thresh = cv2.threshold(working_frame, 45, 255, 0)
				contours, hierarchy = cv2.findContours(thresh, 1, 2)

				# Computes a weighted average for each contour
				averages = self.__weighted_area_average(contours)

				# Selects the best contour by finding the largest average
				largest_average = None
				best_contour = None
				for index in range(len(averages)):
					if (averages[index] > largest_average):
						largest_average = averages[index]
						best_contour = contours[index]

				if (best_contour != None):
					rect = cv2.minAreaRect(best_contour)
					self.__boxes[color] = cv2.cv.BoxPoints(rect)
				else:
					self.__boxes[color] = None

			# Release the reduced color frame that is being held
			self.__image.set_hold_reduced(False)

	def draw_selection(self, colors):
		'''
		Draws the bounding boxes and center points of the selected objects for
		each specified color.
		'''
		if (type(colors) != list):
			raise TypeError("Colors must be passed in a list")

		else:
			for color in colors:
				center = self.get_box_center(color)
				if (center):
					self.__image.draw_point(center)
					self.__image.draw_box(self.__boxes[color])

	def get_box_center(self, color):
		'''
		Finds the center point of the class object's box variable by averaging
		the (x, y) points of the corners. If there is no box, it sets self.box
		to None.
		'''
		box_avg = {}
		box_avg[color] = [0, 0]

		if (self.__boxes[color]):
			for point in range(4):
				for value in range(2):
					box_avg[color][value] = box_avg[color][value] + self.__boxes[color][point][value]
			for value in range(2):
				box_avg[color][value] = (int)(box_avg[color][value] / len(self.__boxes[color]))
			return list(box_avg[color])
		else:
			return None

	def __weighted_area_average(self, contours):
		'''
		Create a weighted average of each box's area and solidity (basically a
		pixel density measurement) based on the following multipliers:
		Area = 0.00015 and Solidity = 0.99985. (This measurement is known as
		'boxocity' in some mechanical engineering circles.)
		'''
		averages = []

		for contour in contours:
			# Calculate the area of the contour
			area = cv2.contourArea(contour)

			# Calculate the density of the contour
			hull = cv2.convexHull(contour)
			hull_area = cv2.contourArea(hull)
			if (hull_area > 0 and area > 0):
				solidity = float(area) / hull_area
			else:
				solidity = 0
			averages.append((area * 0.00015) + (solidity * 0.99985))

		return averages
