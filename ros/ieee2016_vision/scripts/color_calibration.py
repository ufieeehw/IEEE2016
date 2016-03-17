#!/usr/bin/python2
#=============================================================================
# Project: IEEE 2016 Hardware Team Robot (Shia LaBot)
# Module: Color Calibration												v1.1
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import json
import os
import rospkg
import numpy as np


class CalibrationData():
	'''
	Manages the calibrations for image color extraction and distance
	calculation. These are stored in a file in the same directory as the
	script.
	'''
	def __init__(self, file = None):
		self.colors = {}

		# Allows a calibration file to be passed in
		if (file):
			self.calibration_file = file

		# Defaults to a specific calibration file from the ROS path
		else:
			rospack = rospkg.RosPack()
			self.calibration_file = os.path.join(rospack.get_path("ieee2016_vision"), "scripts/color_calibrations.json")

		# Loads the file if it exists; otherwise, creates a new file
		if (os.path.isfile(self.calibration_file)):
			self.load()
		else:
			self.new()

	def new(self):
		'''
		Initializes a new calibration file at the chosen location with blank
		values or zeros depending on the field.
		'''
		self.hsv_ranges = {}
		self.selection_boxes = {}
		self.overlap_prevention_rules = {}
		self.update()

	def save(self):
		'''
		Saves the current calibrations to the JSON file
		'color_calibrations.json' in the same directory as this script.
		'''
		data = {"hsv_ranges": self.hsv_ranges, "selection_boxes": self.selection_boxes, "overlap_prevention_rules": self.overlap_prevention_rules}

		with open(self.calibration_file, 'w') as file:
			json.dump(data, file)

	def load(self):
		'''
		Loads the calibrations from the JSON file 'color_calibrations.json' in
		the same directory as this script and stores them in this class object.
		'''
		with open(self.calibration_file, 'r') as file:
			data = json.load(file)
		self.hsv_ranges = data["hsv_ranges"]
		self.selection_boxes = data["selection_boxes"]
		self.overlap_prevention_rules = data["overlap_prevention_rules"]

		self.update()

	def update(self):
		'''
		Updates the list of available colors and converts the new calibrations
		from an array format to a numpy format that cv2 can use.
		'''
		self.available_colors = self.hsv_ranges.keys()
		for color in self.available_colors:
			self.colors[color] = ((np.array(self.hsv_ranges[color][0])), (np.array(self.hsv_ranges[color][1])))

	def delete(self, color):
		'''
		Deletes a calibration from all of the object's dictionaries.
		'''
		if color in self.hsv_ranges:
			del self.hsv_ranges[color]
		if color in self.selection_boxes:
			del self.selection_boxes[color]
		if color in self.overlap_prevention_rules:
			del self.overlap_prevention_rules[color]
		if color in self.colors:
			del self.colors[color]
		if color in self.available_colors:
			self.available_colors.remove(color)
