#!/usr/bin/env python
import cv2
# import rospy
# import roslib
import numpy as np
import os.path
# roslib.load_manifest('object_detection')
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

'''
#############################################################
#                IEEE Hardware Team 2016                    #
#   Created by:     Nicolas Fry                             #
#   Email:          nicolascoding@ufl.edu                   #
#                                                           #
#   Created for:    The use of detecting images with a      #
#                   Cascade Classifier given a trained file #
#   Created Date:   September 27th, 2015                    #
#                                                           #
#   Modified by:    Ausin Seber                             #
#   Modified Date:  Oct. 06th, 2015                         #
#   Reason Mod.:    Refactoring code to AGILE style         #
#############################################################
'''

def GenericCascade(image, classifierPath, erosionFactor = 0.05):

    if os.path.isfile(classifierPath) is not True:
        print '\n\n***********!!!No training file present\n\n'
        return;

    loadedCascadeClassifier = cv2.CascadeClassifier(classifierPath)

    greyimage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    greyimage = cv2.equalizeHist(greyimage)

    #Look at greyscale image and find occurances of object create a locatedcount variable for debug purposes
    located = loadedCascadeClassifier.detectMultiScale(greyimage,scaleFactor=5.5,minNeighbors=48,minSize=(38,38), flags = cv2.CASCADE_SCALE_IMAGE)

    # Now we go through each rectangle in located
    # and add it to a list of images
    # We also add a bit of extra space to all
    # images based on erosionFactor

    images = []

    for (x, y, w, h) in located:
        erosionW = int(round(w - w*(1 - erosionFactor)))
        erosionH = int(round(h - h*(1 - erosionFactor)))
        rectX = x - erosionW
        rectY = y - erosionH
        rectW = x + w + erosionW
        rectH = y + h + erosionH
        croppedImage = image[rectX:rectY, rectW:rectH]

        images.append(croppedImage)

    return images

def main():
    #Put Path to Cascade locally until os.path is implemented
    cascade_path = '../CascadeRC1/cascade.xml'
    camera = cv2.VideoCapture(-1)

    while(True):

        _, image = camera.read()
        images = GenericCascade(image, cascade_path)
        print len(images)
        cv2.waitKey(50)
        cv2.imshow("Image window", image)


    #Destroy open windows to prevent memory leaks
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
