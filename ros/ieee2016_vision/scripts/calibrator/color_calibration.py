#!/usr/bin/python2
#==============================================================================
# Project: Machine Vision - Color Detection
# Module: Color Calibration												v2.2
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import os
import yaml

from detection import Image, ObjectDetector
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
		Creates and initializes a new color calibration with the specified
		color name and some sane defaults (based on the display image size in
		[width, height]).
		'''
		if (not str.isalnum(color)):
			raise ValueError("The entered name is not alphanumeric")

		elif (color in self.get_available_colors()):
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
			del self.__hsv_ranges[color]
			del self.__selection_boxes[color]
			del self.__overlap_prevention_rules[color]

	def get_hsv_range(self, color):
		'''
		Returns the HSV range for the specified color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		else:
			return self.__hsv_ranges[color]

	def set_hsv_range(self, color, values):
		'''
		Sets the HSV range  values for the specified color.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		elif (len(values) != 3):
			raise ValueError("Maximum, minimum, and value average must be passed to set the HSV range")

		elif (type(values[0]) != list or type(values[1]) != list):
			raise TypeError("The maximum and minimum must be specified as follows: [H, S, V]")

		elif (type(values[2]) != int):
			raise TypeError("The value average must be specified as an integer in the range [0, 256)")

		elif (values[2] < 0 or values[2] >= 256):
			raise ValueError("The value average must be within the range [0, 256)")

		for index in range(2):
			for value in values[index]:
				if (value < 0 or value >= 256):
					raise ValueError("The HSV values must be within the range [0, 256)")

		self.__hsv_ranges[color] = values

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

		elif (type(box_points) != list):
			raise ValueError("The points must be specified in a list as follows: [x1, x2, y1, y2]")

		elif (len(box_points) != 4):
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

		elif (rule in self.__overlap_prevention_rules[color]):
			print("Warning: There is already a prevention rule for '%s' in the calibration for '%s'" % (rule, color))

		else:
			self.__overlap_prevention_rules[color].append(rule)

	def remove_prevention_rule(self, color, rule):
		'''
		Removes the rule color from the list of prevention rules for the
		specified color if it exists.
		'''
		if (not color in self.get_available_colors()):
			raise KeyError("No color calibration exists for '%s'" % (color))

		elif (not rule in self.__overlap_prevention_rules[color]):
			print("Warning: There is no prevention rule for '%s' in the calibration for '%s'" % (rule, color))

		else:
			self.__overlap_prevention_rules[color].remove(rule)


class ColorCalibrator():
	'''
	This class contains all of the methods needed to generate a calibration
	based on the selected area in a frame. Specifically, it ensures that the
	range will lead to selecting nothing in the prevention selection boxes, the
	center point is kept within the specified selection box, and determines the
	average saturation and value over the hue range for balancing later frames.
	'''
	def __init__(self, camera, calibration_file, image_width, gui_calibrator = None):
		self.__camera = camera
		self.__calibration_file = calibration_file

		# The number of frames to average for object detection
		self.__averaging = 16

		# The manager object if the GUI is being used (not required)
		self.__gui = gui_calibrator

		self.__color = ""
		self.__range = None
		self.__value_avg = None

	def get_averaging(self):
		'''
		Returns the averaging value for color calibration.
		'''
		return self.__averaging

	def get_step_range(self):
		'''
		Returns the HSV range for the last detection step.
		'''
		if (self.__gui and self.__step_range):
			return self.__step_range

	def get_step_progress(self):
		'''
		Returns the overall progress of the calibrator.
		'''
		if (self.__gui and self.__step_progress):
			return self.__step_progress

	def get_step_frame(self):
		'''
		Returns the detection frame for the last detection step.
		'''
		if (self.__gui):

			# Pull the requested frame to display the image on
			self.__image.set_hold_reduced(True)
			display_type = self.__gui.get_display_type()
			if (display_type == "unfiltered"):
				self.__image.resize()
			elif (display_type == "reduced"):
				self.__image.reduce_colors(16)
			elif ((display_type == "extracted") and self.__color):
				self.__image.extract_color(self.__color)
			self.__image.set_hold_reduced(False)

			# Draw the detection bounding box and center point if one exists
			self.__detect.draw_selection([self.__color])

			# Reformat the image to RGB, which is what QImage takes, and emit an update signal
			self.__image.reformat_to_rgb()
			step_frame = self.__image.get_frame()

			return step_frame

	def calibrate(self, color):
		self.__gui.status_bar.showMessage("Starting calibration for '%s'" % (color))

		# Creates image and detection objects separate from any that exist
		self.__image = Image(self.__camera, self.__calibration_file, image_width)
		self.__detector = ObjectDetector(image)


	def __calculate_hsv_range(self, box):
		'''
		Slices the box from the array and calculates the maximum and minimum
		HSV values to two standard deviations from the mean on either side.
		'''
		frame = self.__image.get_frame()
		sliced_box = frame[box[0]:box[1], box[2]:box[3]]
		sliced_box = sliced_box.reshape((-1, 3))

		# Calculates the mean and standard deviation of the H, S, and V
		means = np.mean(sliced_box, axis = 0)
		std = np.std(sliced_box, axis = 0)

		# Sets the range to one standard deviation above and below the mean
		h_range = [means[0] - std[0], means[0] + std[0]]
		s_range = [means[1] - std[1], means[1] + std[1]]
		v_range = [means[2] - std[2], means[2] + std[2]]
		self.__range = [[h_range[0], s_range[0], v_range[0]], [h_range[1], s_range[1], v_range[1]]]

	def __eliminate_overlap(self):
		pass

	def __filter_background(self):
		pass

	def __calculate_average_v(self):
		'''
		Calculates the average value for all pixels in which the hue and
		saturation fall within the range. The range MUST be generated first or
		this will not work!
		'''
		if (self.__range):
			frame = self.__image.get_frame()
			pixels = frame.reshape((-1, 3))

			within_min = np.ma.MaskedArray(frame[:, 2], mask = (np.ones_like(frame[:, 0]) * ((frame[:, 0] < self.__range[0][0]))))
			within_max = np.ma.MaskedArray(within_min, mask = (np.ones_like(frame[:, 0]) * ((frame[:, 0] > self.__range[1][0]))))

			self.__value_avg = np.mean(within_max)
