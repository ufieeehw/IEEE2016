#!/usr/bin/python2
#=============================================================================
# Project: Machine Vision - Color Detection
# Module: Color Calibration												v1.4
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import os
import yaml

import numpy as np


class CalibrationData():
	'''
	Manages the calibrations for image color extraction and distance
	calculation. These are stored in a file in the same directory as the
	script.
	'''
	def __init__(self, file, mode):
		self.colors = {}

		# Allows a calibration file to be passed in
		self.file = file

		# Loads the file or creates a new one based on the mode parameter
		if (mode == "new"):
			self.new()
		else:
			self.load()

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
		Saves the current calibrations to the YAML file that was specified.
		'''
		data = {"hsv_ranges": self.hsv_ranges, "selection_boxes": self.selection_boxes, "overlap_prevention_rules": self.overlap_prevention_rules}

		with open(self.file, 'w') as file:
			yaml.safe_dump(data, file)

	def load(self):
		'''
		Loads the calibrations from the YAML file specified and stores them in
		this class object.
		'''
		with open(self.file, 'r') as file:
			data = yaml.safe_load(file)
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
