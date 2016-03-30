#!/usr/bin/python2
#==============================================================================
# Project: Machine Vision - Color Detection
# Module: Camera														v2.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import cv2


# The device id of the camera to use
CAPTURE_DEVICE = 1


class Camera():
	'''
	An object that streams camera data to the image manipulation object. This
	can be replaced with something that better integrates with your software
	stack so long as it provides the activate, deactivate, and get_image
	functions.
	'''
	def __init__(self):
		self.__is_active = False

	def activate(self):
		'''
		Opens a stream from the capture device and sets the state to active.
		Running activate while the camera is active will not recreate the
		stream thread.
		'''
		if (not self.__is_active):
			self.__stream = cv2.VideoCapture(CAPTURE_DEVICE)
			self.__stream.set(3, 1920)
			self.__stream.set(4, 1080)

			# Waits until the camera starts returning frames
			while (self.__stream.read()[1] == None):
				continue

			self.__is_active = True

	def deactivate(self):
		'''
		Releases the camera device and set the state to inactive.
		'''
		if (self.__is_active):
			self.__stream.release()

	def get_frame(self):
		'''
		Returns the frame that is currently stored in the cache if the camera
		is active.
		'''
		if (self.__is_active):
			return self.__stream.read()[1]

		else:
			raise IOError("Cannot retrieve a frame because the camera is not active.")
