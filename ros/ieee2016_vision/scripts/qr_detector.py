#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Header
from sensor_msgs.msg import Image as Image_msg, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from ieee2016_msgs.msg import BlockStamped
from camera_manager import Camera
from point_intersector import PointIntersector

import zbar
import Image
import cv2
import numpy as np
import time
import os

rospack = rospkg.RosPack()
TEMPLATES_FOLDER = os.path.join(rospack.get_path('ieee2016_vision'), 'scripts/templates/')
QR_DISTANCES = [30]

def nothing(x):
    pass

class DetectQRCodeZBar(object):
    """

    Given an image to process, a max amount of codes to detect and a timeout (in seconds),
    this class will attempt to detect as many QR codes as it can and return
    the color and center location of each code as well as a total number of
    codes found.

    This relies on the zbar python package.

    """
    def __init__(self):#, camera):
        # ROS inits for image updating
        self.block_pub = rospy.Publisher("/camera/block_detection", BlockStamped, queue_size=50)

        #rospy.init_node('QR_caller', anonymous=True)
        #self.camera = camera
        self.timeout = timeout

        self.detected_codes = []
        self.image = cv2.imread("qr.jpg")
        #cv2.imwrite("image.jpg",self.image))

    def normal_scan(self, timeout):
        qr_code_count = 0

        start_time = time.time()

        variable_change = 1
        while not rospy.is_shutdown():
            print time.time() - start_time
            if time.time() - start_time > timeout:
                print "Timeout."
                break

            # Create local copy of the image since this is faster to work with and so it doesnt get overwritten
            image = np.copy(self.image)
            #cv2.imwrite("image.jpg",image)

            if image.size > 2: image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

            #Create the identity filter, but with the 1 shifted to the right!
            kernel = np.zeros( (variable_change,variable_change), np.float32)
            kernel[variable_change/2,variable_change/2] = 2.0   #Identity, times two! 

            #Create a box filter:
            boxFilter = np.ones( (variable_change, variable_change), np.float32) / variable_change**2

            #Subtract the two:
            kernel = kernel - boxFilter

            variable_change += 2

            image = cv2.filter2D(image, -1, kernel)

            codes = self.detect_qr(image)
            for code in codes:
                center = ((code.location[0][0] + code.location[2][0])/2,
                          (code.location[0][1] + code.location[2][1])/2)
                cv2.circle(image, center, 20, 0, -1)
                self.publish_block(center, code.data,0)

            # cv2.imshow('codes', image)
            # cv2.waitKey(1)

        print "Time to complete:", time.time() - start_time
    
        return self.detected_codes

    def publish_block(self,uv_point,color,offset):
        print "block detected:",uv_point
        #print uv_point
        b_s = BlockStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id="cam_1"
                    ),
                point=uv_point,
                offset=offset,
                color=color,
                rotation_index=0
            )
        self.block_pub.publish(b_s)

    def detect_qr(self, image):
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
        return image

class DetectQRCodeTemplateMethod(object):
    '''
    Object that deals with detecting QR codes in an image using a template matching method.

    The pros to this are that it will work in various different lighting conditions and works very reliably and very quickly.
    The cons are that template matching is very scale/rotationaly intolerant meaning that we have to be at preset distances.

    Pass in a list of distances (cm) we will be operating at. Note that the template files must exist at that distance. 
    This object is able to create rotated images from a base template image.
    '''
    def __init__(self, distances):
        # ROS inits
        self.block_pub = rospy.Publisher("/camera/block_detection", BlockStamped, queue_size=50)

        # Template Lists
        self.colors = []

        # Used for visual servoing
        #self.point_intersector = PointIntersector()

        # Make sure distances are [full block, back half block, full block, ....]
        self.load_templates(distances)
        self.distances = distances

        self.blocks = []

    def load_templates(self, dists):
        # Only works with one distance right now
        for dist in dists:
            blues = []
            reds = []
            greens = []
            yellows = []

            self.base_path = os.path.join(TEMPLATES_FOLDER, str(dist)) #str(min(QR_DISTANCES, key=lambda x:abs(x-dist))))

            colors = {"/blue":blues,"/red":reds,"/green":greens,"/yellow":yellows}
            thetas = [355,356,357,358,359,0,1,2,3,4,5,
                     85,86,87,88,89,90,91,92,93,94,95,
                     175,176,177,189,180,181,182,183,184,185,
                     265,266,267,268,269,270,271,272,273,274,275]

            # Save raw image for manually getting templates from. Need to specify a camera for this.
            #cv2.imwrite(self.base_path+"/raw.jpg",self.camera.image)

            # Check if there is already a folder with rotated and scaled images in it 
            if cv2.imread(self.base_path+"/blue/0.jpg") is None:
                # No folder exists so lets make some
                # Check to make sure template images are there (we assume if the blue one is there, they all are).
                if cv2.imread(self.base_path+"/blue.jpg") is None:
                    # No folder exists so lets make some
                    print "Template images are missing!"
                    exit()

                print "Rotated templates do not exist. Creating them now."
                self.make_rotated_templates(colors,thetas)

            for color in colors:
                for theta in thetas:
                    colors[color].append(cv2.imread(self.base_path+color+"/"+str(theta)+".jpg",0))

            self.colors.append(colors)

    def make_rotated_templates(self, colors, thetas):
        for color in colors:
            # Make missing folders
            if not os.path.exists(self.base_path+color):
                os.makedirs(self.base_path+color)

            # Load template and rotate its
            template = cv2.imread(self.base_path+color+".jpg",0)
            rows,cols = template.shape

            # Do some filtering on the template image
            blur = cv2.GaussianBlur(template,(5,5),2)
            template = cv2.addWeighted(template,1.5,blur,-.5,0)
            #template = cv2.equalizeHist(template)
            for theta in thetas:
                rot_mat = cv2.getRotationMatrix2D((cols/2,rows/2),theta,1)
                cv2.imwrite(self.base_path+color+"/"+str(theta)+".jpg",cv2.warpAffine(template,rot_mat,(cols,rows)))

    def match_templates(self, camera, args):
        '''
        **Depreciated**

        Called by main program to process an image from the specified camera and to find QR Codes in that image.
        
        This method takes in the distance to the blocks. We will probably work at 2-4 different distances. Close up and far
            away for both full blocks and the back half blocks.

        This method will publish BlockStamped messages that can be recived by the processing node.

        Derectives and special arguments are as follows (distances are from the camera to the wall):
            inital_scan: perform normal scan for blocks .29 m away.
                args: distance_to_wall
            half_block_scan: perform half block scan for blocks .3525 m away.
                args: distance_to_wall
            close_up_scan: Scan for a single block to line ourselves up with at .095 m away.
                args: [distance_to_wall, block_color, qr_rotation]. These are specified so we don't have to search every color and every rotation.
        '''

        self.threshold = .65
        self.camera = camera
        self.normal_scan(args)

        # if directive == "inital_scan":
        #     #distance_to_wall = args - cam_to_base_link
        #     self.normal_scan(args)
        # elif directive == "half_block_scan":
        #     #distance_to_wall = args - cam_to_base_link
        #     self.normal_scan(args, offset=.0625)
        # elif directive == "close_up_scan":
        #     #distance_to_wall = args - cam_to_base_link[0]
        #     self.visual_servo(distance_to_wall)

    def normal_scan(self, camera, dist, frames=8):
        '''
        Uses template matching to test all combinations of colors and rotation for the specified dist and for the number of frames.

        Offset is used to test for halfblocks.

        The detection method is kind of weird - we take the first frame, scan it for blue qr codes then take the next frame and scan for red
        etc etc until we've reached the fourth frame, then we restart again until weve checked 'frames' frames. This was done to keep image
        up to date.
        '''
        self.threshold = .63
        
        if dist == "full":
            dist = 30
            offset = 0
        elif dist == "half":
            dist = 36.25
            offset = .0625
        
        self.camera = camera

        colors = self.colors[self.distances.index(dist)]
        blues = colors["/blue"]  
        reds = colors["/red"]
        greens = colors["/green"]
        yellows = colors["/yellow"]
        image = np.copy(self.camera.image)

        temp = np.zeros(image.shape[:2])

        # Run this whole process frames/4 times
        for frames in range(frames/4):
            # The process is detecing qr codes. 
            for frame_count in range(4):
                # Load image from camera, save it
                image = np.copy(self.camera.image)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                gray = cv2.equalizeHist(gray)

                if frame_count == 0:
                    mid_point = np.array(blues[0].shape[::-1])/2
                    for rotation,template in enumerate(blues):
                        # Apply matching threshold for this rotation
                        res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                        loc = np.where( res >= self.threshold)
                        #temp += res
                        for pt in zip(*loc[::-1]):
                            self.publish_block(pt+mid_point,"blue",offset,rotation)
                            #cv2.circle(image, (pt[0] + mid_point[0],pt[1] + mid_point[1]), 15, (255,0,0), -1)
                    #print tmp
                    #cv2.imshow("img", temp)
                    #cv2.waitKey(0)
                    #temp = np.zeros(image.shape[:2])

                elif frame_count == 1:
                    mid_point = np.array(reds[0].shape[::-1])/2
                    for rotation,template in enumerate(reds):
                        # Apply matching threshold for this rotation
                        res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                        loc = np.where( res >= self.threshold)
                        for pt in zip(*loc[::-1]):
                            self.publish_block(pt+mid_point,"red",offset,rotation)
                            #cv2.circle(image, (pt[0] + mid_point[0],pt[1] + mid_point[1]), 15, (0,0,255), -1)

                elif frame_count == 2:
                    mid_point = np.array(greens[0].shape[::-1])/2
                    for rotation,template in enumerate(greens):
                        # Apply matching threshold for this rotation
                        res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                        loc = np.where( res >= self.threshold)
                        for pt in zip(*loc[::-1]):
                            self.publish_block(pt+mid_point,"green",offset,rotation)
                            #cv2.circle(image, (pt[0] + mid_point[0],pt[1] + mid_point[1]), 15, (0,255,0), -1)

                elif frame_count == 3:
                    mid_point = np.array(yellows[0].shape[::-1])/2
                    for rotation,template in enumerate(yellows):
                        # Apply matching threshold for this rotation
                        res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                        loc = np.where( res >= self.threshold)
                        for pt in zip(*loc[::-1]):
                            self.publish_block(pt+mid_point,"yellow",offset,rotation)
                            #cv2.circle(image, tuple(pt+mid_point), 15, (0,255,255), -1)
                # Only for displaying
                #cv2.imshow("found",image)
                #cv2.waitKey(1)

    def visual_servo(self, dist, block_color, orientation, threshold=20, timeout=5):
        '''
        Publishes a twist in order to move the robot and line ourselves up with the block in question.
        We specify a color and orientation so we don't have to check every combination of color and rotation when template matching.
        
        Orientation is an index to the color list NOT an absolute rotation.
        '''
        color_name = "/"+block_color
        this_color = self.colors[self.distances.index(dist)][color_name]

        image = np.copy(camera.image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        image_center = image.shape[1]/2
        print "CENTER:",image_center


        delta = threshold + 1
        start_time = time.time()
        while delta > threshold and time.time() - start_time > timeout:

            template_mid_point = np.array(this_color[0].shape[::-1])/2
            template = this_color[orientation]

            # Apply matching threshold for this rotation
            res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
            loc = np.where( res >= self.threshold)

            # We are going to average and display all the points here.
            pt = np.mean(loc[::-1], axis=1) + template_mid_point
            delta = pt[0] - image_center
            print delta

        #cv2.circle(image, (_pt[0] + mid_point[0],pt[1] + mid_point[1]), 15, (255,0,0), -1)    

    def publish_block(self,uv_point,color,offset,rotation):
        print "block detected:",uv_point
        #print uv_point
        b_s = BlockStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self.camera.name
                    ),
                point=uv_point,
                offset=offset,
                color=color,
                rotation_index=rotation
            )
        self.block_pub.publish(b_s)
        
if __name__ == "__main__":
    #d = DetectQRCodeZBar(qr_code_count=1, timeout=10)
    rospy.init_node("detect_qr")
    cam = Camera(1)
    cam.activate()
    d = DetectQRCodeZBar()
    d.normal_scan(3)
    # d = DetectQRCodeTemplateMethod([30])#50,56.25])
    # r = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     d.normal_scan(cam, 'full')

    #     r.sleep()
    #sd.continuous_publish()
    #     r.sleep()
    # rospy.spin()
    #cv2.waitKey(0)