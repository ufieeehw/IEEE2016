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
		self.colors = {}
		self.load_calibration()

	def save_calibration(self):
		'''
		Saves the current calibrations to the JSON file
		'color_calibrations.json' in the same directory as this script.
		'''
		with open('color_calibrations.json', 'w') as file:
			json.dump(self.calibrations, file)

	def load_calibration(self):
		'''
		Loads the calibrations from the JSON file 'color_calibrations.json' in
		the same directory as this script and stores them in this class object.
		'''
		# Imports the dictionary and determines what colors are in it
		with open('color_calibrations.json', 'r') as file:
			self.calibrations = json.load(file)
		self.available_colors = self.calibrations.keys()

		# Converts the values in the dictionary to numpy arrays for cv2
		for color in self.available_colors:
			self.colors[color] = ((np.array(self.calibrations[color][0])), (np.array(self.calibrations[color][1])))


class Image():
	'''
	Contains various custom image manipulation methods. Some methods require
	the selection of a color to proceed. Colors id's are as follows: red,
	green, blue, and yellow.
	'''
	def __init__(self, camera, calibration, width = 640):
		# Baseline operating parameters
		self.camera = camera
		self.calibration = calibration
		self.frame = self.camera.image
		self.operating_image_width = width
		self.hold_redux_frame = False

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

		 	# Blurs the remaining colors to reduce noise
		 	self.frame = cv2.GaussianBlur(working_frame, (5, 5), 0)

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
		mask = cv2.inRange(working_frame, self.calibration.colors[color][0], self.calibration.colors[color][1])
		self.frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)


class ObjectDetection():
	'''
	Generates usable data about the location of objects within the frame.
	Colors id's are as follows: red, green, blue, and yellow.
	'''
	def __init__(self, camera, calibration):
		# Baseline operating parameters
		self.image = Image(camera, calibration, 320)
		self.box = {}
		self.box_center = {}

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
				self.box[color] = cv2.cv.BoxPoints(rect)
			else:
				self.box[color] = None

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
				if (self.box[color]):
					for point in range(4):
						for value in range(2):
							box_avg[color][point][value] = box_avg[color][point][value] + self.box[color][point][value]

				# Aborts the averaging if a set number of measurements return None
				elif (retry[color] == max_retry):
					self.box[color] = None
					break

				# Attempts to retry failed measurements
				else:
					retry[color] += 1
					run -= 1

		for color in colors:
			# Divides the sums of the points' (x, y) values by the amount of values
			if (self.box[color]):
				for point in range(4):
					for value in range(2):
						box_avg[color][point][value] = box_avg[color][point][value] / amount
				self.box[color] = box_avg[color]

	def get_box_center(self):
		'''
		Finds the center point of the class object's box variable by averaging
		the (x, y) points of the corners. If there is no box, it sets self.box
		to None.
		'''
		box_avg = {}

		for color in self.box.keys():
			box_avg[color] = [0, 0]

			if (self.box[color]):
				for point in range(4):
					for value in range(2):
						box_avg[color][value] = box_avg[color][value] + self.box[color][point][value]
				for value in range(2):
					box_avg[color][value] = (int)(box_avg[color][value] / len(self.box[color]))
				self.box_center[color] = tuple(box_avg[color])
			else:
				self.box_center[color] = None


def train_box_order(camera, colors, averaging):
	'''
	Used to perform the locations and order of the colored train boxes at the
	beginning of a run.
	'''
	box_centers = []
	order = []

	camera.activate()
	calibration = Calibration()
	detection = ObjectDetection(camera, calibration)

	# Finds the average center of the boxes around the largest object
	detection.average_box(detection.select_largest_object, colors , averaging)
	detection.get_box_center()

	for color in colors:
		# Stores the center's x positions with reference to color and in a list
		if (detection.box_center[color]):
			box_centers.append(detection.box_center[color][0])

		# Prints an error and returns None if unsuccessful for a color
		else:
			print("ERROR: color_detection could not detect the center of the %s box - aborting" % (color))
			return None

	# Arranges the list by value and applys the order to a list of colors
	box_centers.sort()
	for center in box_centers:
		for color in colors:
			if (detection.box_center[color][0] == center):
				order.append(color)

	camera.deactivate()

	# Returns a list of colors from left to right in the frame
	return order

def debug_selection(camera, colors, averaging):
	'''
	Used to debug the selection of objects based on hue and relative size in
	the frame. Displays real-time selection output using cv2 for feedback.
	'''
	camera.activate()
	calibration = Calibration()
	image = Image(camera, calibration, 320)
	detection = ObjectDetection(camera, calibration)

	while (True):
		# Finds the center of the box around the largest object
		detection.average_box(detection.select_largest_object, colors, averaging)
		detection.get_box_center()

		# Pulling values used to render the debugging image
		image.resize()
		frame = image.frame
		box = detection.box
		box_center = detection.box_center

		for color in colors:
			# Draw a box around the selected object in the captured frame
			if (box[color]):
				box_to_draw = np.int0(box[color])
				cv2.drawContours(frame, [box_to_draw], 0, (0, 0, 255), 2)

			# Draw the center point of the box in the captured frame
			if (box_center[color]):
				cv2.circle(frame, box_center[color], 3, (0, 255, 0))

		# Display the frame for debugging
		cv2.imshow('Debugging', frame)
		if (cv2.waitKey(5) == 27):
			cv2.destroyAllWindows
			break

	camera.deactivate()

if __name__ == "__main__":
	rospy.init_node("color_dection")
	camera = Camera(1)
	# print(train_box_order(camera, ["red", "blue", "yellow"], 8))
	debug_selection(camera, ["blue"], 10)
	exit()
