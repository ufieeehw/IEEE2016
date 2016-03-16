#!/usr/bin/python2
#=============================================================================
# Project: IEEE 2016 Hardware Team Robot (Shia LaBot)
# Module: Color Calibration												v1.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import json
import os
import rospkg
import numpy as np


class Calibrations():
	'''
	Manages the calibrations for image color extraction and distance
	calculation. These are stored in a file in the same directory as the
	script.
	'''
	def __init__(self, file = None):
		# Allows a calibration file to be passed in
		if (file):
			self.calibration_file = file
		else:
			rospack = rospkg.RosPack()
			self.calibration_file = os.path.join(rospack.get_path("ieee2016_vision"), "scripts/color_calibrations.json")

		self.colors = {}
		self.load()

	def save(self):
		'''
		Saves the current calibrations to the JSON file
		'color_calibrations.json' in the same directory as this script.
		'''
		with open(self.calibration_file, 'w') as file:
			json.dump(self.calibrations, file)

	def load(self):
		'''
		Loads the calibrations from the JSON file 'color_calibrations.json' in
		the same directory as this script and stores them in this class object.
		'''
		with open(self.calibration_file, 'r') as file:
			self.calibrations = json.load(file)
		self.available_colors = self.calibrations.keys()
		self.load_cv2()

	def load_cv2(self):
		'''
		Converts the loaded calibrations from an array format to a numpy format
		that cv2 can use.
		'''
		for color in self.available_colors:
			self.colors[color] = ((np.array(self.calibrations[color][0])), (np.array(self.calibrations[color][1])))

	def delete(self, color):
		'''
		Deletes a calibration from all of the object's dictionaries.
		'''
		if color in self.calibrations:
			del self.calibrations[color]
		if color in self.colors:
			del self.colors[color]
		if color in self.available_colors:
			self.available_colors.remove(color)
