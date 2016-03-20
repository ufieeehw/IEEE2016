#!/usr/bin/python2
#=============================================================================
# Project: Machine Vision - Color Detection
# Module: Camera Stream												v1.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

import threading
import time

import cv2


# The device id of the camera to use
CAPTURE_DEVICE = 1


class Camera():
	'''
	An object that streams camera data to the image manipulation object. This
	can be replaced with something that better integrates with your software
	stack so long as it provides the activate and deactivate functions and the
	self.image variable.
	'''
	def __init__(self, fps = 60):
		self.image = None
		self.loop_read = False
		self.fps = fps

		self.stream_thread = threading.Thread(target = self.get_frame)
		self.stream_thread.daemon = True

	def activate(self):
		'''
		Opens a stream from the capture device and starts a thread to read from
		it at the desired FPS rate. Running activate while the camera is active
		will not recreate the stream thread.
		'''
		if (not self.stream_thread.isAlive()):
			self.stream = cv2.VideoCapture(CAPTURE_DEVICE)

			self.loop_read = True
			self.stream_thread = threading.Thread(target = self.get_frame)
			self.stream_thread.daemon = True
			self.stream_thread.start()

			while (self.image == None):
				continue

	def deactivate(self):
		'''
		Soft kills the stream thread, waits for it to stop, and releases the
		camera device
		'''
		if (self.stream_thread.isAlive()):
			self.loop_read = False
			while (self.stream_thread.isAlive()):
 				continue

			self.stream.release()

	def get_frame(self):
		'''
		Retrieves frames from the video stream at the defined FPS.
		'''
		while (self.loop_read):
			ret, self.image = self.stream.read()
			time.sleep(1.0 / self.fps)
