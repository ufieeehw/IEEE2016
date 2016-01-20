#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

import argparse
import cv2
import numpy as np
import time
import os

class Map():
    def __init__(self, map_topic, map_name):
        rospy.init_node('map_server')

        self.map_topic = map_topic
        self.map_name = map_name
        self.height = 0
        self.width = 0

        self.done_saving = False

    def publish_map(self):
        map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=2)
        rate = rospy.Rate(10)

        meta_data = []

        #try:
        with open(os.path.join(os.path.dirname(__file__), 'map/' + self.map_name + '.txt')) as f:
            meta_data = f.read().splitlines()
        
        self.width = int(meta_data[2])
        self.height = int(meta_data[3])

        map_meta_data = MapMetaData(    
            map_load_time=rospy.Time.now(),
            resolution=float(meta_data[1]),
            width=self.width,
            height=self.height,
            origin=Pose(
                position=Point(
                    x=float(meta_data[4]),
                    y=float(meta_data[5]),
                    z=float(meta_data[6]),
                ),
                orientation=Quaternion(
                    x=float(meta_data[7]),
                    y=float(meta_data[8]),
                    z=float(meta_data[9]),
                    w=float(meta_data[10]),
                ),
            )
        )

        # Read image and convert back to occ grid
        image = cv2.imread(os.path.join(os.path.dirname(__file__), 'map/' + self.map_name + '.bmp'),0).astype(int).flatten()
        
        #cv2.imshow("map_raw",image)
        #cv2.imshow("map_crop",self.find_bounds(image))
        #self.find_bounds(image)

        image = 100*image/255.0 - 1 
        rospy.loginfo("Publishing Map.")
        o = OccupancyGrid(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map",
                    ),
                info=map_meta_data,
                data=image.tolist()
            )
        print o.header
        while not rospy.is_shutdown():
            map_pub.publish(o)
            rate.sleep()

        #except:
        #    rospy.logwarn("Map data missing or currupt.")
        
    def save_map(self):
        # For calling from outside the object
        map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.save_to_disk,queue_size=2)
        rospy.loginfo("Waiting for map...")
        rospy.spin()

    def save_to_disk(self,msg):
        # Acutally manages saving the recieved map
        if not self.done_saving:
            self.done_saving = True
            rospy.loginfo("Map found!")

            width = msg.info.width
            height = msg.info.height

            # Read map data and generate an image based on it. 
            image = np.array(msg.data).reshape(height,width)
            # Find the optimal size to save the image as
            top_max, bottom_max, left_max, right_max = 0,height-1,0,width-1
            for i,row in enumerate(image):
                if np.count_nonzero(row+1) > 0:
                    top_max = i-1
                    break
            for i,row in enumerate(image[::-1]):
                if np.count_nonzero(row+1) > 0:
                    bottom_max = bottom_max - i + 1 
                    break
            for i,row in enumerate(image.T):
                if np.count_nonzero(row+1) > 0:
                    left_max = bottom_max - i + 1 
                    break
            image = (image + 1)/100.0 * 255

            # Save the image as bmp which is lossless
            cv2.imwrite("map/"+self.map_name+".bmp",image)

            #print msg.info
            meta_data = ["Map Metadata"]
            meta_data.append(str(msg.info.resolution))
            meta_data.append(str(msg.info.width))
            meta_data.append(str(msg.info.height))
            meta_data.append(str(msg.info.origin.position.x))
            meta_data.append(str(msg.info.origin.position.y))
            meta_data.append(str(msg.info.origin.position.z))
            meta_data.append(str(msg.info.origin.orientation.x))
            meta_data.append(str(msg.info.origin.orientation.y))
            meta_data.append(str(msg.info.origin.orientation.z))
            meta_data.append(str(msg.info.origin.orientation.w))
            #print meta_data
            with open("map/"+self.map_name+".txt", "w") as f:
                for l in meta_data:
                    f.write(l)
                    f.write('\n')
            
            rospy.loginfo("Map saved.")
            rospy.signal_shutdown("Saving completed.")


    def find_bounds(self,image):
        ret,thresh = cv2.threshold(image.astype(np.uint8).reshape(self.width,self.height),1,255,0)
        
        contours,hierarchy = cv2.findContours(thresh, 1, 2)
        #cv2.imshow("cropped",thresh)
        #cv2.waitKey(10)
        for c in contours:
            print c
            x,y,w,h = cv2.boundingRect(c)
            print x,y,w,h
            cv2.rectangle(image,(x,y),(x+w,y+h),128,2)

        cv2.imwrite("squares",image)
        
        #cropped_image = image[x:x+h,y:y+w]
        
        #print cropped_image
        #cv2.imshow("cropped",cropped_image)
        #cv2.waitKey(0)

        #return cropped_image


if __name__ == "__main__":
    # Read if user wants to save or publish
    parser = argparse.ArgumentParser(description='Dealing with displaying, saving, and publishing occupancy grids.')
    parser.add_argument('--save', default = False, action="store_true", help='Save map on topic: topic_name.')
    parser.add_argument('topic_name', type=str, help="Will be the topic saved from or published to, depending on --save.")
    parser.add_argument('map_name', type=str, help="Name of the map to load or save.")
    args = parser.parse_args()

    m = Map(args.topic_name,args.map_name)
    if args.save:
        m.save_map()
    else:
        m.publish_map()
    print "Done"