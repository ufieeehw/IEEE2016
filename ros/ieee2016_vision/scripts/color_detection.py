#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import Image


# Baseline operating parameters
OPERATING_IMAGE_WIDTH = 640
IMAGE_COLOR_DEPTH = 16

# These are the upper and lower boundaries for each color
# These WILL be replaced by an automatically generated calibration file
LOWER_RED = np.array([0, 180, 180])
UPPER_RED = np.array([20, 255, 255])
LOWER_GREEN = np.array([50, 40, 40])
UPPER_GREEN = np.array([80, 250, 250])
LOWER_BLUE = np.array([80, 80, 80])
UPPER_BLUE = np.array([150, 255, 255])
LOWER_YELLOW = np.array([20, 120, 120])
UPPER_YELLOW = np.array([50, 255, 255])


def resize_image(msg):
	'''
	Resizes a frame from the video feed to the operating image with and
	proportional height.
	'''
	# Attempts to pull a frame from the ROS video stream
	try:
		frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError as error:
		print error

	# Resizes the frame based on the original aspect ratio
	height, width = frame.shape[0], frame.shape[1]
	aspect_ratio = float(width) / height
	new_dimensions = (OPERATING_IMAGE_WIDTH, int(OPERATING_IMAGE_WIDTH / aspect_ratio))
	resized_frame = cv2.resize(frame, new_dimensions, interpolation = cv2.INTER_AREA)

	return(resized_frame)


def reduce_colors(frame):
	'''
	Reduces the range of colors present in a frame of the video feed based on
	the set color depth.
	'''
	# Converts the frame to a format that is usable with K-Means clustering
	working_frame = frame.reshape((-1, 3))
	working_frame = np.float32(working_frame)

	# Use K-Means clustering to identify the 16 most predominant colors
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
	null, label, center = cv2.kmeans(working_frame, IMAGE_COLOR_DEPTH, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

	# Reduces the colors in the original image based on the clustering results
	center = np.uint8(center)
	redux_frame = center[label.flatten()]
	redux_frame = redux_frame.reshape((frame.shape))

	return redux_frame


def color_extraction(frame):
	'''
	Generates four images from the passed frame that contain hues specified
	within the defined boundaries.
	'''
	# Converts the image from RGB to HSV
	working_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# Creates masks for each hue range
	red_mask = cv2.inRange(working_frame, LOWER_RED, UPPER_RED)
	green_mask = cv2.inRange(working_frame, LOWER_GREEN, UPPER_GREEN)
	blue_mask = cv2.inRange(working_frame, LOWER_BLUE, UPPER_BLUE)
	yellow_mask = cv2.inRange(working_frame, LOWER_YELLOW, UPPER_YELLOW)

	return(red_mask, green_mask, blue_mask, yellow_mask)

def testing(msg):
	'''
	Serves as a testing platform for the script, displaying the various stages
	of each processed frame.
	'''
	# The main image processing chain
	frame = resize_image(msg)
	redux_frame = reduce_colors(frame)
	red_mask, green_mask, blue_mask, yellow_mask = color_extraction(redux_frame)

	# Overlays the masks onto the original frame for each hue range
	red_frame = cv2.bitwise_and(frame, frame, mask = red_mask)
	green_frame = cv2.bitwise_and(frame, frame, mask = green_mask)
	blue_frame = cv2.bitwise_and(frame, frame, mask = blue_mask)
	yellow_frame = cv2.bitwise_and(frame, frame, mask = yellow_mask)

	# Display all frames for debugging
	cv2.imshow('Captured Frame', frame)
	cv2.imshow('Reduced Color Depth', redux_frame)
	cv2.imshow('Extracted Red', red_frame)
	cv2.imshow('Extracted Green', green_frame)
	cv2.imshow('Extracted Blue', blue_frame)
	cv2.imshow('Extracted Yellow', yellow_frame)
	cv2.waitKey(5)

rospy.init_node("color_detection")
image_sub = rospy.Subscriber("/cam_1/image_raw", Image, testing)
rospy.spin()
