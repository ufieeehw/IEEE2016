#!/usr/bin/env python

from pycparser.c_ast import Case
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import Image

# Baseline operating parameters
OPERATING_IMAGE_WIDTH = 150
IMAGE_COLOR_DEPTH = 16

# These are the upper and lower boundaries for each color
# These WILL be replaced by an automatically generated calibration file
RED_HUE = (np.array([0, 100, 100]), np.array([20, 255, 255]))
GREEN_HUE = (np.array([50, 42, 42]), np.array([80, 255, 255]))
BLUE_HUE = (np.array([80, 100, 100]), np.array([150, 255, 255]))
YELLOW_HUE = (np.array([20, 110, 110]), np.array([50, 255, 255]))
HUE_RANGES = (RED_HUE, GREEN_HUE, BLUE_HUE, YELLOW_HUE)

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
	ret, label, center = cv2.kmeans(working_frame, IMAGE_COLOR_DEPTH, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

	# Reduces the colors in the original image based on the clustering results
	center = np.uint8(center)
	redux_frame = center[label.flatten()]
	redux_frame = redux_frame.reshape((frame.shape))

 	# Blurs the remaining colors to reduce noise
 	redux_frame = cv2.GaussianBlur(frame, (5, 5), 0)

	return redux_frame


def color_extraction(frame, color):
	'''
	Generates an image from the passed frame that contain hues specified within
	the defined boundaries for the color id passed. Colors id's are as follows:
	red = 0, green = 1, blue = 2, and yellow = 3.
	'''
	# Creates a mask for the selected hue range and overlays them onto the frame
	working_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(working_frame, HUE_RANGES[color][0], HUE_RANGES[color][1])
	color_frame = cv2.bitwise_and(working_frame, working_frame, mask = mask)

	return(color_frame)

def select_largest_object(frame, color):
	'''
	MAKE SURE THIS IS FORMATED CORRECTLY AND ADD A DOC STRING.
	'''
	largest_area = -1
	largest_contour_index = -1

	# Generates a gray frame for the passed color id
	working_frame = color_extraction(frame, color)
 	working_frame = cv2.cvtColor(working_frame, cv2.COLOR_BGR2GRAY)

	# Finds all contours within the frame
	ret, thresh = cv2.threshold(working_frame, 50, 255, 0)
	contours, hierarchy = cv2.findContours(thresh, 1, 2)

	# Selects the largest contour in the frame
	for i in range(len(contours)):
		area = cv2.contourArea(contours[i])
		if (area > largest_area):
			largest_area = area
			largest_contour_index = i
	if (largest_contour_index != -1):
		rect = cv2.minAreaRect(contours[largest_contour_index])
		box = cv2.cv.BoxPoints(rect)

		# Returns the four corner points of the selected object
		return box

	else:
		return -1

def testing(msg):
	'''
	Serves as a testing platform for the script, displaying the various stages
	of each processed frame.
	'''
	# The main image processing chain
	redux_frame = reduce_colors(resize_image(msg))
	box = select_largest_object(redux_frame, 2)

	# Draw a box around the selected object in the captured frame
	if (box != -1):
		box = np.int0(box)
		cv2.drawContours(redux_frame, [box], 0, (0, 0, 255), 2)

	# Display the frames for debugging
	cv2.imshow('Selected Object', redux_frame)
	cv2.waitKey(5)

rospy.init_node("color_detection")
image_sub = rospy.Subscriber("/cam_1/image_raw", Image, testing)
rospy.spin()
