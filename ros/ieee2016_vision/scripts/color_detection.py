#!/usr/bin/env python

from camera_manager import Camera
import cv2
import numpy as np
import rospy


# These are the upper and lower boundaries for each color
# These WILL be replaced by an automatically generated calibration file
RED_HUE = (np.array([0, 100, 100]), np.array([20, 255, 255]))
GREEN_HUE = (np.array([50, 42, 42]), np.array([80, 255, 255]))
BLUE_HUE = (np.array([80, 100, 100]), np.array([150, 255, 255]))
YELLOW_HUE = (np.array([20, 110, 110]), np.array([50, 255, 255]))
HUE_RANGES = (RED_HUE, GREEN_HUE, BLUE_HUE, YELLOW_HUE)


class ColorProcessing():
	'''
	Contains various color operations that are in the structure of a chain
	(e.g. top level methods call the methods that they depend on). Some methods
	require the selection of a color to proceed. Colors id's are as follows:
	red = 0, green = 1, blue = 2, and yellow = 3. Current top level methods are
	as follows: resize_frame, select_largest_object, and average_box.
	'''
	def __init__(self, camera):
		# Baseline operating parameters
		self.operating_image_width = 320
		self.image_color_depth = 16
		self.camera = camera

	def resize_frame(self):
		'''
		Resizes a frame from the video feed to the operating image with and
		proportional height.
		'''
		self.frame = self.camera.image
		height, width = self.frame.shape[0], self.frame.shape[1]
		aspect_ratio = float(width) / height
		new_dimensions = (self.operating_image_width, int(self.operating_image_width / aspect_ratio))
		self.frame = cv2.resize(self.frame, new_dimensions, interpolation = cv2.INTER_AREA)

	def reduce_colors(self):
		'''
		Reduces the range of colors present in a frame of the video feed based on
		the set color depth.
		'''
		self.resize_frame()

		# Converts the frame to a format that is usable with K-Means clustering
		working_frame = self.frame.reshape((-1, 3))
		working_frame = np.float32(working_frame)

		# Use K-Means clustering to identify the 16 most predominant colors
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
		ret, label, center = cv2.kmeans(working_frame, self.image_color_depth, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

		# Reduces the colors in the original image based on the clustering results
		center = np.uint8(center)
		working_frame = center[label.flatten()]
		working_frame = working_frame.reshape((self.frame.shape))

	 	# Blurs the remaining colors to reduce noise
	 	self.frame = cv2.GaussianBlur(working_frame, (5, 5), 0)

	def color_extraction(self, color):
		'''
		Generates an image from the passed frame that contain hues specified within
		the defined boundaries for the color id passed.
		'''
		self.reduce_colors()

		# Creates a mask for the selected hue range and overlays them onto the frame
		working_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(working_frame, HUE_RANGES[color][0], HUE_RANGES[color][1])
		self.frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)

	def select_largest_object(self, color):
		'''
		Selects the largest object in a frame by finding all contours and
		selecting the largest one.
		'''
		largest_area = None
		largest_contour_index = None
		self.color_extraction(color)

		# Generates a gray frame for the passed color id
	 	working_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

		# Finds all contours within the frame
		ret, thresh = cv2.threshold(working_frame, 50, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, 1, 2)

		# Selects the largest contour in the frame
		for i in range(len(contours)):
			area = cv2.contourArea(contours[i])
			if (area > largest_area):
				largest_area = area
				largest_contour_index = i

		if (largest_contour_index):
			rect = cv2.minAreaRect(contours[largest_contour_index])
			self.box = cv2.cv.BoxPoints(rect)
		else:
			self.box = None

	def average_box(self, function, color, amount):
		'''
		Defines a new set of box points that is the result of averaging
		'amount' number of box points from the chosen method (this method
		should set self.box). Will retry a limited number of times before
		simply setting self.box to None.
		'''
		max_retry = amount / 4
		retry = 1
		box_avg = [[0, 0], [0, 0], [0, 0], [0, 0]]

		for i in range(amount):
			# Run the passed function (should be one from this class object)
			function(color)

			# Add up all of the box points' (x, y) values
			if (self.box):
				for j in range(4):
					for k in range(2):
						box_avg[j][k] = box_avg[j][k] + self.box[j][k]

			# Aborts the averaging if a set number of measurements return None
			elif (retry == max_retry):
				self.box = None
				break

			# Attempts to retry failed measurements
			else:
				retry += 1
				i -= 1

		# Divides the sums of the points' (x, y) values by the amount of values
		if (self.box):
			for j in range(4):
				for k in range(2):
					box_avg[j][k] = (box_avg[j][k] / amount)
			self.box = box_avg


if __name__ == "__main__":
	'''
	Serves as a testing platform for the script, displaying the various stages
	of each processed frame.
	'''
	rospy.init_node("color_detection")
	camera = Camera("cam_1")
	camera.activate()
	processing = ColorProcessing(camera)
	while (True):
		processing.average_box(processing.select_largest_object, 2, 8)
		processing.resize_frame()
		box = processing.box
		frame = processing.frame

		# Draw a box around the selected object in the captured frame
		if (box):
			box = np.int0(box)
			cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

		# Display all frames for debugging
		cv2.imshow('Debugging', frame)
		if (cv2.waitKey(5) == 27):
			cv2.destroyAllWindows
			break
	camera.deactivate()
