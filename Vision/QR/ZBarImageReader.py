#!/usr/bin/env python
import cv2
# import rospy
# import roslib
import numpy as np
import os.path
import zbar
from GenericCascade import GenericCascade
# roslib.load_manifest('object_detection')
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

'''
#############################################################
#                IEEE Hardware Team 2016                    #
#   Created by:     Austin Seber                            #
#   Email:          aseber@techsouth.cc                     #
#                                                           #
#   Created for:    Based on work created by Nicholas Fry.  #
#                   Detects and returns  the QRCode based   #
#                   on an input image. Returns a tuple of   #
#                   {image, centerX, centerY,               #
#                   {symbols{type, content}}}               #
#   Created Date:   Oct. 6th, 2015                          #
#                                                           #
#   Modified by:    N/A                                     #
#   Modified Date:  N/A                                     #
#   Reason Mod.:    N/A                                     #
#############################################################
'''

def ZBarImageReader(image):
    #initialize ZBAR by creating a reader
    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')

    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width, channels = image.shape
    imageString = grayImage.tostring()
    zbar_image = zbar.Image(width, height, 'Y800', imageString)

    scanner.scan(zbar_image)
    symbols = []

    for symbol in zbar_image:
        # print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data
        symbols.append({'type':symbol.type, 'data':symbol.data})

    return {'image':image, 'centerX':int(round(width/2)), 'centerY':int(round(height/2)), 'symbols':symbols}

def main():
    #Put Path to Cascade locally until os.path is implemented
    cascade_path = '../CascadeRC1/cascade.xml'
    camera = cv2.VideoCapture(-1)

    while(True):

        _, image = camera.read()
        images = GenericCascade(image, cascade_path, 0.0)
        for currentImage in images:
            tup = ZBarImageReader(image)
            symbols = tup['symbols']
            for symbol in symbols:
                print symbol['data']

        cv2.waitKey(1)
        cv2.imshow("Image window", image)


    #Destroy open windows to prevent memory leaks
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
