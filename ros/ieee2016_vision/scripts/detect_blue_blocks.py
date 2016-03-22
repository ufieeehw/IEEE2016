import os
import cv2
import yaml
import rospkg
import numpy as np
from camera_manager import Camera
from point_intersector import PointIntersector

class DetectBlueBlocks():

	def __init__(self):
		self.CALIBRATION_FILE_URI = os.path.join(rospack.get_path('ieee2016_vision'), 'scripts/color/color_calibrations.yaml')
		self.load_calibration_data(CALIBRATION_FILE_URI)
		self.hsv_ranges = self.load_calibration_data['hsv_ranges']
		self.blue_hsv = hsv_ranges['blue']
		self.lower_blue = self.blue_hsv[0]
		self.upper_blue = self.blue_hsv[1]

	def load_calibration_data(self):
		with open(self.CALIBRATION_FILE_URI, 'r') as infile:
			self.calibration_data = yaml.load(infile)
		print "Data Loaded."
        	
	def detect_blue_blocks(self, camera):
		orig_image = cv2.imread(camera.image, 1)
		dupe = orig_image.copy()

		#Remove noise from the colored image
		dupe = cv2.fastNlMeansDenoisingColored(dupe,None,10,10,7,21)

		#Convert image from BGR to HSV
		hsv = cv2.cvtColor(dupe, cv2.COLOR_BGR2HSV)
		#Set lower and upper bound of blue color value (HSV)
		# lower_bound = np.array([50, 50, 0])
		# upper_bound = np.array([210, 200, 75])

		# Find the HSV colors within the specified boundaries and create a mask of what was detected
		mask = cv2.inRange(hsv, np.array(self.lower_blue), np.array(self.upper_blue))
		output = cv2.bitwise_and(dupe, image, mask = mask)

		#Convert the colors to B/W, blur it, and apply binary threshold
		output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
		output = cv2.GaussianBlur(output, (3, 3), 0)
		ret,output = cv2.threshold(output, 15, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

		#Use Canny detection to find edges
		output = cv2.Canny(output, 30, 250)

		#Morphological transformations on canny image to try to form detectable rectangles
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
		output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel) #MORPH_CLOSE = dilation then erosion

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (33,33))
		kernel2 = kernel
		output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

		#Retains only the boundaries of the detected shape
		output = cv2.morphologyEx(output, cv2.MORPH_GRADIENT, kernel) 

		output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel2)

		#Finds the contours of the image
		_, cnts, _ = cv2.findContours(output.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		centroid = []
		all_coor = []

		# Loop over the contours
		for c in cnts:
		    # Approximate the contour
		    peri = 0.09*cv2.arcLength(c, True)
		    approx = cv2.approxPolyDP(c, peri, True)
		    # print cv2.contourArea(c)

		    # Only draw contours if the area is in the specified range.
		    if 15000 > cv2.contourArea(c) > 8000:
		        # If the approximated contour has four points, then assume that the
		        # contour is a block -- a block is a rectangle and thus has four vertices.
		        if len(approx) == 4:
		            #Draws rectangle around contours
		            cv2.drawContours(image, [approx], -1, (0, 255, 0), 4) 

		            #Calculates the moments of the contours
		            moments = cv2.moments(approx)

		            #Finds the x and y coordinates of the quadrilateral's centroid
		            cx = int(moments['m10']/moments['m00'])
		            cy = int(moments['m01']/moments['m00'])

		            #Store the coordinates into a list
		            centroid.append(cx)
		            centroid.append(cy)

		            #Put a dot where the centroid is
		            cv2.circle(image, tuple(centroid), 10, (0, 255, 0), -1)

		            #Create a list for all found centroids
		            all_coor.append(centroid)
		            centroid = []

		# print all_coor

		#Sort all center coordinates by their x-coordinate (ascending)
		for x in all_coor:
		    all_coor = sorted(all_coor, key = lambda x: x[0], reverse=False)

		if len(all_coor) == 0:
			# print 'No blocks detected'
			pt = None
		elif len(all_coor) == 1:
			pt = all_coor[0]    
		else:    

    		# Variables that store the two left-most blocks
			left_most = all_coor[0]
			left_most2 = all_coor[1]

	    	#Finds the left-most block or the upper left-most block if it exists
	    	if abs(left_most[0] - left_most2[0]) >= 100:
	        	pt = left_most
	    	elif left_most[1] > left_most2[1]:
	        	pt = left_most2

		i = PointIntersector()  	
		return i.intersect_point(camera, pt)