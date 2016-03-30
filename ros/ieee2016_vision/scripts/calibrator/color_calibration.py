#!/usr/bin/python2
#==============================================================================
# Project: Machine Vision - Color Detection
# Module: Color Calibration												v2.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import os
import yaml

import numpy as np


class CalibrationFile():
	'''
	Manages the calibrations for image color extraction and distance
	calculation. These are stored in a file in the same directory as the
	script.
	'''
	# The minimum area for a selection in pixels
	__minimum_selection = 400

	def __init__(self, file, mode, image_shape):
		self.__colors = {}

		# Allows a calibration file to be passed in
		self.__file = file

		# Allows the image shape to be passed in for selection box managment
		self.__image_shape = image_shape

		# Will create the specified file to load if it does not exist
		if (not os.path.isfile(file)):
			mode = "new"

		# The class will fail to initialize if the image is too small
		if ((image_shape[0] * image_shape[1]) < (self.__minimum_selection * 9)):
			raise ValueError("A %d x %d image is too small to use for calibration" % (self.__image_shape[0], self.__image_shape[1]))
			del self

		if (mode == "new"):
			self.__new()
		elif (mode == "load"):
			self.__load()

	def __new(self):
		'''
		Initializes a new calibration file at the chosen location with blank
		values or zeros depending on the field.
		'''
		self.__hsv_ranges = {}
		self.__selection_boxes = {}
		self.__overlap_prevention_rules = {}

	def __load(self):
		'''
		Loads the calibrations from the YAML file specified and stores them in
		this class object.
		'''
		with open(self.__file, 'r') as file:
			data = yaml.safe_load(file)
		self.__hsv_ranges = data["hsv_ranges"]
		self.__selection_boxes = data["selection_boxes"]
		self.__overlap_prevention_rules = data["overlap_prevention_rules"]

	def save(self):
		'''
		Saves the current calibrations to the YAML file that was specified.
		'''
		data = {"hsv_ranges": self.__hsv_ranges, "selection_boxes": self.__selection_boxes, "overlap_prevention_rules": self.__overlap_prevention_rules}

		with open(self.__file, 'w') as file:
			yaml.safe_dump(data, file)

	def get_path(self):
		'''
		Returns a global path to the calibration file this object is using.
		'''
		return self.__file

	def get_available_colors(self):
		'''
		Returns a list of the available color calibrations.
		'''
		return self.__hsv_ranges.keys()

	def create_color(self, color):
		'''
		Creates and initializes a new color calibration with the specified name
		and some sane defaults (based on the display image size in
		[width, height]).
		'''
		if (not str.isalnum(name)):
			raise ValueError("The entered name is not alphanumeric")

		elif (name in self.get_available_colors()):
			raise KeyError("A calibration for this color already exists")

		else:
			x1 = int(self.__image_shape[0] * (1.0 / 3))
			x2 = int(self.__image_shape[0] * (2.0 / 3))
			y1 = int(self.__image_shape[1] * (1.0 / 3))
			y2 = int(self.__image_shape[1] * (2.0 / 3))

			self.__hsv_ranges[color] = [[0, 0, 0], [0, 0, 0]]
			self.__selection_boxes[color] = [x1, x2, y1, y2]
			self.__overlap_prevention_rules[color] = []

	def delete_color(self, color):
		'''
		Deletes a calibration from all of the object's dictionaries.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		else:
			del self.hsv_ranges[color]
			del self.selection_boxes[color]
			del self.overlap_prevention_rules[color]

	def get_hsv_range(self, color):
		'''
		Returns the HSV range for the specified color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		else:
			return self.__hsv_ranges[color]

	def set_hsv_range(self, color, range):
		'''
		Sets the HSV range for the specified color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		elif (type(range) != list):
			raise ValueError("Both a maximum and minimum value must be passed to set the HSV range")

		elif (type(range) != list):
			raise ValueError("The maximum and minimum must be specified as follows: [H, S, V]")

		for value in range[0]:
			if (value < 0 or point >= 256):
				raise ValueError("The HSV values must be within the range [0, 256)")
		for value in range[1]:
			if (value < 0 or point >= 256):
				raise ValueError("The HSV values must be within the range [0, 256)")

		self.__hsv_ranges[color] = range

	def get_np_hsv_range(self, color):
		'''
		Returns the HSV range for the specified color in a format that np and
		cv2 can use.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		else:
			return [np.array(self.__hsv_ranges[color][0]), np.array(self.__hsv_ranges[color][1])]

	def get_minimum_selection(self):
		'''
		Returns the minimum size for a selection box.
		'''
		return self.__minimum_selection

	def get_selection_box(self, color):
		'''
		Returns the selection box for the specified color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		else:
			return self.__selection_boxes[color]

	def set_selection_box(self, color, box_points):
		'''
		Sets the selection box for the specified color if the box points were
		formatted properly.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		elif (not type(box_points) != list):
			raise ValueError("The points must be specified in a list as follows: [x1, x2, y1, y2]")

		elif (not len(box_points) != 4):
			raise ValueError("The points must be specified in a list as follows: [x1, x2, y1, y2]")

		width = abs(box_points[1] - box_points[0])
		height = abs(box_points[3] - box_points[2])
		area = width * height
		if (area < self.__minimum_selection):
			raise ValueError("A selection of %d pixels is too small for accurate calibration" % (area))

		for point in range(0, 2):
			if (point < 0 or point >= self.__image_shape[0]):
				raise ValueError("One of the X points specified were outside of the image boundaries")
		for point in range(2, 4):
			if (point < 0 or point >= self.__image_shape[1]):
				raise ValueError("One of the Y points specified were outside of the image boundaries")

		self.__selection_boxes[color] = box_points

	def get_overlap_prevention(self, color):
		'''
		Returns the overlap prevention rules for the specified color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		else:
			return self.__overlap_prevention_rules[color]

	def add_prevention_rule(self, color, rule):
		'''
		Adds the rule color to the list of prevention rules for the specified
		color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		elif (not rule in self.__overlap_prevention_rules[color]):
			print("Warning: There is no prevention rule for '%s' in the calibration for '%s'" % (rule, color))

		else:
			self.__overlap_prevention_rules[color].append(rule)

	def remove_prevention_rule(self, color, rule):
		'''
		Removes the rule color from the list of prevention rules for the
		specified color if it exists.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		elif (rule in self.__overlap_prevention_rules[color]):
			print("Warning: There is already a prevention rule for '%s' in the calibration for '%s'" % (rule, color))

		else:
			self.__overlap_prevention_rules[color].remove(rule)
