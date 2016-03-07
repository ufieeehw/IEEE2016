#!/usr/bin/python2

import json

from camera_manager import Camera
import cv2
import numpy as np
from point_intersector import PointIntersector
import rospy


class Calibration():
	'''
	Manages the calibrations for image color extraction and distance
	calculation. These are stored in a file in the same directory as the
	script.
	'''
	def __init__(self):
		self.load_calibration()

	def save_calibration(self):
		'''
		Saves the current calibrations to the JSON file
		'color_calibrations.json' in the same directory as this script.
		'''
		pass

	def load_calibration(self):
		'''
		Loads the calibrations from the JSON file 'color_calibrations.json' in
		the same directory as this script and stores them in this class object.
		'''
		# These are the upper and lower boundaries for each color
		# These WILL be replaced by an automatically generated calibration file
		self.red = (np.array([0, 100, 100]), np.array([20, 255, 255]))
		self.green = (np.array([50, 42, 42]), np.array([80, 255, 255]))
		self.blue = (np.array([80, 100, 100]), np.array([150, 255, 255]))
		self.yellow = (np.array([20, 110, 110]), np.array([50, 255, 255]))


class Image():
	'''
	Contains various custom image manipulation methods. Some methods require
	the selection of a color to proceed. Colors id's are as follows: red,
	green, blue, and yellow.
	'''
	def __init__(self, camera, width = 640):
		# Baseline operating parameters
		self.camera = camera
		self.frame = self.camera.image
		self.operating_image_width = width

	def resize(self):
		'''
		Resizes a frame from the video feed to the operating image with and
		proportional height.
		'''
		self.frame = self.camera.image
		height, width = self.frame.shape[0], self.frame.shape[1]
		aspect_ratio = float(width) / height
		new_dimensions = (self.operating_image_width, int(self.operating_image_width / aspect_ratio))
		self.frame = cv2.resize(self.frame, new_dimensions, interpolation = cv2.INTER_AREA)

	def reduce_colors(self, image_color_depth):
		'''
		Reduces the range of colors present in a frame of the video feed based on
		the set color depth.
		'''
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

	 	# Blurs the remaining colors to reduce noise
	 	self.frame = cv2.GaussianBlur(working_frame, (5, 5), 0)

	def extract_color(self, color):
		'''
		Generates an image from the passed frame that contain hues specified within
		the defined boundaries for the color id passed.
		'''
		self.reduce_colors(16)

		# Creates a mask for the selected hue range and overlays them onto the frame
		working_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(working_frame, color[0], color[1])
		self.frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)


class ObjectDetection():
	'''
	Generates usable data about the location of objects within the frame.
	Colors id's are as follows: red, green, blue, and yellow.
	'''
	def __init__(self, camera):
		# Baseline operating parameters
		self.image = Image(camera)
		self.image.operating_image_width = 320
		self.box = None
		self.box_center = None

	def select_largest_object(self, color):
		'''
		Selects the largest object in a frame by finding all contours and
		selecting the largest one.
		'''
		largest_area = None
		largest_contour_index = None
		self.image.extract_color(color)

		# Generates a gray frame for the passed color id
	 	working_frame = cv2.cvtColor(self.image.frame, cv2.COLOR_BGR2GRAY)

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

	def get_box_center(self):
		'''
		Finds the center point of the class object's box variable by averaging
		the (x, y) points of the corners. If there is no box, it sets self.box
		to None.
		'''
		box_avg = [0, 0]

		if (self.box):
			for j in range(4):
				for k in range(2):
					box_avg[k] = box_avg[k] + self.box[j][k]
			for k in range(2):
				box_avg[k] = (int)(box_avg[k] / 4)
			self.box_center = tuple(box_avg)
		else:
			self.box_center = None


class ColorBox():
	'''
	Used to perform the locations and order of the color drop boxes at the
	beginning of a run. Calculates a 3D point on the map for each box as well
	as attempting to double check the distance to it.
	'''
	def __init__(self, camera):
			self.detection = ObjectDetection(camera)
			self.point = PointIntersector()
			self.colors = ("red", "green", "blue", "yellow")

	def get_map_location(self, color):
		'''
		Uses point_intersector to determine the position of a box on the
		robot's map relative to it's position in the camera's plane.
		'''
		pass

	def verify_distance(self, color):
		'''
		Calculates the distance to a box based on it's size in the frame. If it
		is not within an acceptable range of the value obtained from
		point_intersector, the two values are averaged.
		'''
		pass

	def set_waypoint(self, color):
		'''
		Creates new waypoints for each of the boxes by color based on the
		position values obtained by the object.
		'''
		pass

	def repeat_for_colors(self, function):
		'''
		Repeats the specified function for each of the colors set in the
		initialization of the class object.
		'''
		pass


if __name__ == "__main__":
	'''
	Serves as a testing platform for the script, displaying the various stages
	of each processed frame.
	'''
	rospy.init_node("color_detection")
	camera = Camera(1)
	camera.activate()
	image = Image(camera, 320)
	detection = ObjectDetection(camera)
	calibration = Calibration()

	while (True):
		# The actual processing of the image
		detection.average_box(detection.select_largest_object, calibration.yellow, 8)
		detection.get_box_center()

		# Pulling values used to render the debugging image
		image.resize()
		frame = image.frame
		box = detection.box
		box_center = detection.box_center

		# Draw a box around the selected object in the captured frame
		if (box):
			box = np.int0(box)
			cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

		# Draw the center point of the box in the captured frame
		if (box_center):
			cv2.circle(frame, box_center, 3, (0, 255, 0))

		# Display the frame for debugging
		cv2.imshow('Debugging', frame)
		if (cv2.waitKey(5) == 27):
			cv2.destroyAllWindows
			break

	camera.deactivate()
