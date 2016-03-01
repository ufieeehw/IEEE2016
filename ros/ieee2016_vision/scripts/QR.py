import rospy
import rospkg
from sensor_msgs.msg import Image as Image_msg, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


import QR
import zbar
import Image
import cv2
import numpy as np
import time
import os

rospack = rospkg.RosPack()
TEMPLATES_FOLDER = os.path.join(rospack.get_path('ieee2016_vision'), 'scripts/templates/')
QR_DISTANCES = [57]

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
    def __init__(self, qr_code_count, timeout, camera):
        # ROS inits for image updating
        #rospy.init_node('QR_caller', anonymous=True)
        self.camera = camera
        self.qr_code_count_max = qr_code_count
        self.timeout = timeout

        self.detected_codes = []
        self.image = camera.image
        cv2.imwrite("image.jpg",self.image)
        self.begin_processing()

    def begin_processing(self):
        # Process the class image (with multiple threads?) to find all qr codes. The image can be updated
        qr_code_count = 0

        # The mask acts to remove already detected QR codes so they arent repeateitly detected
        mask = np.zeros(self.image.shape[:2], dtype=np.uint8)+255
        #cv2.rectangle(mask,(0,0),(1080,900),0,-1)

        sliders = np.zeros((10,500,3), np.float32)
        cv2.namedWindow('sliders')

        cv2.createTrackbar('H_L','sliders',0,255,nothing)
        cv2.createTrackbar('H_U','sliders',0,255,nothing)
        cv2.createTrackbar('S_L','sliders',0,255,nothing)
        cv2.createTrackbar('S_U','sliders',0,255,nothing)
        cv2.createTrackbar('V_L','sliders',0,255,nothing)
        cv2.createTrackbar('V_U','sliders',0,255,nothing)

        start_time = time.time()
        while qr_code_count < self.qr_code_count_max and not rospy.is_shutdown():
            print time.time() - start_time
            # if time.time() - start_time > self.timeout:
            #     print "Timeout."
            #     break

            # Create local copy of the image since this is faster to work with and so it doesnt get overwritten
            image = self.image
            #cv2.imwrite("image.jpg",image)
            """
            Convert to Black and white, blur, threshold, sharpen, erode then apply the mask. 
            (Code modified from online resources)
            """
            if image.size > 2: image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            #image = cv2.GaussianBlur(image,(3,3),0)
            #image = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3)
             # Convert BGR to HSV
            #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #hsv = cv2.bilateralFilter(hsv,3,75,75)

            # #cv2.imshow('sliders',sliders)

            #lower = np.array([cv2.getTrackbarPos('H_L','sliders'),cv2.getTrackbarPos('S_L','sliders'),cv2.getTrackbarPos('V_L','sliders')])
            #upper = np.array([cv2.getTrackbarPos('H_U','sliders'),cv2.getTrackbarPos('S_U','sliders'),cv2.getTrackbarPos('V_U','sliders')])
            # #lower = np.array([27,0,0])
            # #upper = np.array([239,53,255])

            #Bitwise-AND mask and original image
            #remove = cv2.inRange(hsv, lower, upper)
            #image = cv2.bitwise_and(image,image, mask=remove)
            #image = np.delete(image, (0,1), axis=2)
            image = cv2.equalizeHist(image)
            #image = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,3,3)


            image = image & mask

            # Display image
            cv2.imshow('sliders',sliders)
            cv2.imshow("img", cv2.resize(image, (0,0), fx=0.8, fy=0.8)); cv2.waitKey(100)

            color,coor = self.detect_qr(image)
            if color == None or coor == None: 
                #print "nothing found"
                continue
        
            #print coor
            center = ((coor[0][0] + coor[2][0])/2,
                      (coor[0][1] + coor[2][1])/2)

            #point = self.convert_to_3d_points(coor[0],coor[2])

            #self.detected_codes.append([color,point])
            qr_code_count += 1
            print "CODES FOUND:",qr_code_count, color
            #print "found",color,center

            # Draw mask around detected QR code as to not repeat
            cv2.circle(mask, center, 20, 0, -1)

        print "Time to complete:", time.time() - start_time
    
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

    def convert_to_3d_points(self, point_1, point_2):
        if not self.camera_info_received: print "ERROR"
        
        # Paramters
        QR_side_length = 3.64 #cm
        diagonal = np.sqrt(2)*QR_side_length
        #print self.proj_mat
        proj_mat_pinv = np.linalg.pinv(np.array([self.proj_mat]).reshape((3,4)))

        # Genereate projection rays through points 1 and 2
        ray_1 = proj_mat_pinv.dot(np.append(point_1,1).reshape((3,1))).reshape((1,4))[0]
        ray_2 = proj_mat_pinv.dot(np.append(point_2,1).reshape((3,1))).reshape((1,4))[0]

        #print ray_1,ray_2
        #print np.dot(ray_1,ray_2)

        # Angle between two rays
        mag_1, mag_2 = np.sqrt(ray_1.dot(ray_1)), np.sqrt(ray_2.dot(ray_2))
        theta = np.arccos(np.dot(ray_1,ray_2)/(mag_1 * mag_2))    

        dist = (diagonal / 2.0) / (np.arctan(theta / 2.0))

        # Calculate unit vector to center point
        mid_point = [(point_1[0] + point_2[0])/2,
                      (point_1[1] + point_2[1])/2]

        vect = proj_mat_pinv.dot(np.append(mid_point,1).reshape((3,1))).reshape((1,4))[0][:3]
        #print vect,dist
        point = vect * dist / np.linalg.norm(vect)

        return poin

class DetectQRCodeTemplateMethod(object):
    def __init__(self, qr_code_count, camera):
        # ROS inits for image updating
        #rospy.init_node('QR_caller', anonymous=True)
        self.camera = camera
        self.qr_code_count_max = qr_code_count
        #self.timeout = timeout

        # Template Lists
        self.blues = []
        self.reds = []
        self.greens = []
        self.yellows = []

        self.detected_codes = []

        self.load_templates(57)
        self.match_templates()


    def load_templates(self,dist):
        # Find the closest distance folder (the folders are in cm)
        self.base_path = os.path.join(TEMPLATES_FOLDER, str(dist)) #str(min(QR_DISTANCES, key=lambda x:abs(x-dist))))

        colors = {"/blue":self.blues,"/red":self.reds,"/green":self.greens,"/yellow":self.yellows}
        thetas = [0,90,180,270]

        # Save raw image
        cv2.imwrite(self.base_path+"/raw.jpg",self.camera.image)

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

    def match_templates(self):
        threshold = .65
        group_distance = 20
        
        blues = self.blues 
        reds = self.reds 
        greens = self.greens 
        yellows = self.yellows 


        frame_count = 0
        while not rospy.is_shutdown():
            found = 0
            image = self.camera.image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            
            existing_points = []

            mid_point = self.blues[0].shape[::-1]

            if frame_count == 0:
                for template in self.blues:
                    # Apply matching threshold for this rotation/scale
                    mid_point = template.shape[::-1]
                    res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                    loc = np.where( res >= threshold)
                    for pt in zip(*loc[::-1]):
                        # Check if the found point already exists. This should be optimized and made to use an average of centers rather 
                        # then just the first one it finds.
                        exists = False
                        # for existing_point in existing_points:
                        #     if np.linalg.norm(np.array(existing_point) - np.array(pt)) < group_distance:
                        #         exists = True
                        #         break
                        
                        if not exists:
                            found+=1
                            existing_points.append(pt)
                            cv2.circle(image, (pt[0] + mid_point[0]/2,pt[1] + mid_point[1]/2), 15, (255,0,0), -1)
                    #print existing_points
            elif frame_count == 1:
                existing_points = []
                for template in self.reds:
                    # Apply matching threshold for this rotation/scale
                    #mid_point = template.shape[::-1]
                    res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                    loc = np.where( res >= threshold)
                    for pt in zip(*loc[::-1]):
                        # Check if the found point already exists. This should be optimized and made to use an average of centers rather 
                        # then just the first one it finds.
                        exists = False
                        # for existing_point in existing_points:
                        #     if np.linalg.norm(np.array(existing_point) - np.array(pt)) < group_distance:
                        #         exists = True
                        #         break
                        
                        if not exists:
                            found+=1
                            existing_points.append(pt)
                            cv2.circle(image, (pt[0] + mid_point[0]/2,pt[1] + mid_point[1]/2), 15, (0,0,255), -1)
                    #print existing_points
            elif frame_count == 2:
                existing_points = []
                for template in self.greens:
                    # Apply matching threshold for this rotation/scale
                    #mid_point = template.shape[::-1]
                    res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                    loc = np.where( res >= threshold)
                    for pt in zip(*loc[::-1]):
                        # Check if the found point already exists. This should be optimized and made to use an average of centers rather 
                        # then just the first one it finds.
                        exists = False
                        # for existing_point in existing_points:
                        #     if np.linalg.norm(np.array(existing_point) - np.array(pt)) < group_distance:
                        #         exists = True
                        #         break
                        
                        if not exists:
                            found+=1
                            existing_points.append(pt)
                            cv2.circle(image, (pt[0] + mid_point[0]/2,pt[1] + mid_point[1]/2), 15, (0,255,0), -1)
                    #print existing_points
                    existing_points = []

            elif frame_count == 3:
                for template in self.yellows:
                    # Apply matching threshold for this rotation/scale
                    #mid_point = template.shape[::-1]
                    res = cv2.matchTemplate(gray,template,cv2.TM_CCOEFF_NORMED)
                    loc = np.where( res >= threshold)
                    for pt in zip(*loc[::-1]):
                        # Check if the found point already exists. This should be optimized and made to use an average of centers rather 
                        # then just the first one it finds.
                        exists = False
                        # for existing_point in existing_points:
                        #     if np.linalg.norm(np.array(existing_point) - np.array(pt)) < group_distance:
                        #         exists = True
                        #         break
                        
                        if not exists:
                            found+=1
                            existing_points.append(pt)
                            cv2.circle(image, (pt[0] + mid_point[0]/2,pt[1] + mid_point[1]/2), 15, (0,255,255), -1)
                    #print existing_point

            frame_count += 1
            if frame_count > len(self.blues): frame_count = 0
            print frame_count
            cv2.imshow("found",image)
            cv2.waitKey(1)