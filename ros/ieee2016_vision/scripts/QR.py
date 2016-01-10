import QR
import zbar
import Image
import cv2
import numpy as np
import time

import rospy
from sensor_msgs.msg import Image as Image_msg
from cv_bridge import CvBridge, CvBridgeError

class DetectQRCode(object):
    """

    Given an image to process, a max amount of codes to detect and a timeout (in seconds),
    this class will attempt to detect as many QR codes as it can and return
    the color and center location of each code as well as a total number of
    codes found.

    This relies on the zbar python package.

    """
    def __init__(self, qr_code_count, timeout, image_topic):
        # ROS inits for image updating
        rospy.init_node('QR_caller', anonymous=True)
        self.image_sub = rospy.Subscriber(image_topic,Image_msg,self.image_recieved)

        self.qr_code_count_max = qr_code_count
        self.timeout = timeout

        self.detected_codes = []

        self.image = 0
    
    def image_recieved(self,msg):
        print "Frame updated"
        try:
            self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

    def begin_processing(self):
        # Process the class image (with multiple threads?) to find all qr codes. The image can be updated
        
        start_time = time.time()
        while self.image is 0:
            print "No image found. Waiting for image."
            time.sleep(1)
            if time.time() - start_time > self.timeout:
                print "Timeout."
                return 0
        qr_code_count = 0

        # The mask acts to remove already detected QR codes so they arent repeated
        mask = np.zeros(self.image.shape[:2], dtype=np.uint8)+255

        start_time = time.time()
        while qr_code_count < self.qr_code_count_max:
            print time.time() - start_time
            if time.time() - start_time > self.timeout:
                print "Timeout."
                break

            # Create local copy of objects image so it doesnt get updated halfway through
            image = self.image

            """
            Convert to Black and white, blur, threshold, sharpen, erode then apply the mask. 
            (Code modified from online resources)
            """
            if image.size > 2: image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            image = cv2.GaussianBlur(image,(3,3),0)
            image = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,17,5)

            #Create the identity filter, but with the 1 shifted to the right!
            kernel = np.zeros((9,9), np.float32)
            kernel[4,4] = 2.0   #Identity, times two! 

            #Create a box filter:
            boxFilter = np.ones((9,9), np.float32) / 81.0

            #Subtract the two:
            kernel = kernel - boxFilter

            #Note that we are subject to overflow and underflow here...but I believe that
            # filter2D clips top and bottom ranges on the output, plus you'd need a
            # very bright or very dark pixel surrounded by the opposite type.
            image = cv2.filter2D(image, -1, kernel)

            kernel = np.ones((1.1,1.1),np.uint8)
            image = cv2.erode(image,kernel)
            
            image = image & mask

            cv2.imshow("img", image)
            cv2.waitKey(1)


            color,coor = self.detect_qr(image)
            if color == None or coor == None: 
                print "nothing found"
                continue
        
            print coor
            center = ((coor[0][0] + coor[2][0])/2,
                      (coor[0][1] + coor[2][1])/2)

            print self.detected_codes

            self.detected_codes.append([color,center])
            qr_code_count += 1

            print "found",color,center
            cv2.rectangle(mask,coor[0],coor[2],0,-1)

        print "Time to complete:", time.time() - start_time
        
        self.image_sub.unregister()
        return self.detected_codes

    def detect_qr(self, image):
        orig_img = image

        # create a reader
        scanner = zbar.ImageScanner()

        # configure the reader
        scanner.parse_config('enable')

        # obtain image data
        pil = Image.fromarray(image)
        width, height = pil.size
        raw = pil.tostring()

        # wrap image data
        image = zbar.Image(width, height, 'Y800', raw)

        # scan the image for barcodes
        scanner.scan(image)

        # extract results
        for symbol in image:
            # do something useful with results
            if symbol.data is not "None":
                return symbol.data,symbol.location

        # If nothing was found, return Nones
        return None,None