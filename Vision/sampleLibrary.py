import cv2
import numpy

'''
#############################################################
#   		     IEEE Hardware Team 2016                    #
#	Created by: Nicolas Fry									#
#	Email:		nicolascoding@ufl.edu						#
#															#
#   Created for:    Sample format for future demo algorithms#
#   Created Date:	September 27th, 2015  					#
#															#
#   Modified by: Nicolas Fry								#
#	Modified date: September 27th, 2015						#
#	Reason:		Insert reason for modification				#
#############################################################
'''

def SampleTemplate(image):

	'''
	Given-when-then is an agile TESTING method, but it can be used in software development as well
	It organizes the code where we have something 'Given to us', something we have to do, then 
	'''

	#When I apply image manipulations such as canny edge detection
	edges = cv2.Canny(image, 100, 200)


	#Then I output what is needed
	cv2.imshow("SampleWindow", edges)
	cv2.waitKey(0)


def main():
	#initialize class

	#Given I have a  OpenCV image frame 
	imageSource = cv2.imread("../blockImageRepository/yellow/longblock/TopView.JPG")

	SampleTemplate(imageSource)

	#close openCv windows gracefully
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 