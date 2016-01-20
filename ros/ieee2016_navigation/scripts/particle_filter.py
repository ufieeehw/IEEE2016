#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from sensor_msgs.msg import LaserScan
import tf

import cv2
import os
import numpy as np
import math
import time
import random

PARTICLE_COUNT = 100

class Map():
    def __init__(self, map_name):
        # load map from bmp file in map/ folder
        self.map = cv2.imread(os.path.join(os.path.dirname(__file__), 'map/' + map_name + '.bmp'),0).astype(np.uint8)
        self.pub = rospy.Publisher('sim_scan', LaserScan, queue_size=10)
        #rospy.init_node('particle_filter', anonymous=True)

        # Load some meta data about the map
        with open(os.path.join(os.path.dirname(__file__), 'map/' + map_name + '.txt')) as f:
            meta_data = f.read().splitlines()

        self.cell_resolution = float(meta_data[1]) #m/px
        print self.cell_resolution
        self.width = int(meta_data[2])
        self.height = int(meta_data[3])
        self.origin = [self.width/2,self.height/2]


    def simulate_scan(self, *point):
        # Point should be in x,y,heading
        # 0,0,0 is the middle of the map pointed right
        point += np.array([self.origin[0],self.origin[1],0])

        ret,thresh = cv2.threshold(self.map,127,255,cv2.THRESH_BINARY)

        # Parameters of simulated laserscan (should come from a real lidar scan)
        angle_increment = math.radians(180)/500.0
        min_angle = math.radians(-90) #rads
        max_angle = math.radians(90) #rads
        max_range =  5 #meters
        min_range = .02 #meters
        ranges = np.zeros(int((max_angle-min_angle)/angle_increment))

        # Temp image to store simulated laserscan results
        lines = np.zeros(thresh.shape)
        for i in range(int((max_angle-min_angle)/angle_increment)):
            # Draw a test point from 'point' location out every 'angule_increment' rads from min_range to max_range
            # Once this test point intersects a wall in the map, save the data and create a LaserScan message with the data
            theta = min_angle + i*angle_increment

            # Make test points along lines from origin and test if there is a wall at each point along the line
            # Only check for points between max and min range
            for r in range(int(min_range/self.cell_resolution),
                           int(max_range/self.cell_resolution)):
                x = point[0] + int(r * np.cos(theta+point[2]))
                y = point[1] + int(r * np.sin(theta+point[2]))
                
                #If the point is off the map, break
                if x >= self.width or y >= self.height: break
                if x <= 0 or y <= 0: break

                #lines[y,x] = 255

                # If the test point is a wall, add the dist to the list and break
                if thresh[y,x] == 255:
                    dist = math.sqrt((abs(point[0]-x))**2 + (abs(point[1]-y))**2)*self.cell_resolution
                    ranges[i] = dist
                    #lines[x,y] = 255
                    break
                #cv2.imshow("lines",lines)
                #cv2.waitKey(1)

            #cv2.imshow("thresh",thresh)
            #cv2.waitKey(1)
        self.pub.publish(LaserScan(    
            header=Header(
                stamp = rospy.Time.now(),
                frame_id = "map",
                ),
            angle_min=min_angle,
            angle_max=max_angle,
            angle_increment=angle_increment,
            time_increment=0,
            scan_time=0,
            range_min=min_range,
            range_max=max_range,
            ranges=ranges.tolist(),
            intensities=[],
            )
        )

class Particle():
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def update_pos(self,dx,dy,dhead):
        self.x += dx
        self.y += dy
        self.heading += dhead

    def return_pose(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.heading)
        p = Pose(
            position=Point(
                    x=self.x,
                    y=self.y,
                    z=0
                ),
            orientation=Quaternion(
                    x=q[0],
                    y=q[1],
                    z=q[2],
                    w=q[3],
                )
        )
        return p

class Filter():
    def __init__(self, p_count, center, radius, heading_range):
        # Pass the max number of particles, the center and radius of where inital particle generation will be (meters), and the range of heading values (min,max)
        # ROS Inits
        rospy.init_node('particle_filter', anonymous=True)
        self.test_points_pub = rospy.Publisher('test_points', PoseArray, queue_size=2)
        self.odom_pub = 
        # Some arguemnts that wont need to change for Shia

        # Generate random point in circle and add to list
        self.particles = [] 
        for p in range(p_count):
            # random angle
            alpha = 2 * math.pi * random.random()
            # random radius
            r = radius * random.random()
            # calculating coordinates
            x = r * math.cos(alpha) + center[0]
            y = r * math.sin(alpha) + center[1]

            # Generate random heading
            heading = random.uniform(heading_range[0], heading_range[1])

            self.particles.append(Particle(x,y,heading))

    def publish_particle_array(self):
        pose_arr = []
        for p in self.particles:
            pose_arr.append(p.return_pose())
            
        self.test_points_pub.publish(PoseArray(
            header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map",
                ),
            poses=pose_arr,
            )
        )

def nothing(x):
    pass

f = Filter(PARTICLE_COUNT, (1,0), .5, (.1,-.1))

r = rospy.Rate(1)
while not rospy.is_shutdown():
    f.publish_particle_array()
    r.sleep()

# m = Map("ieee_small")
# map_image = m.map

# # Create a black image, a window
# cv2.namedWindow('bars')

# # create trackbars for color change
# cv2.createTrackbar('x','bars',0,300,nothing)
# cv2.createTrackbar('y','bars',0,300,nothing)
# cv2.createTrackbar('theta','bars',0,628,nothing)

# r = rospy.Rate(10)
# while not rospy.is_shutdown():
#     x = cv2.getTrackbarPos('x','bars')
#     y = cv2.getTrackbarPos('y','bars')
#     t = cv2.getTrackbarPos('theta','bars')
    
#     image = map_image
#     #cv2.circle(image,(x,y),2,255,-1)

#     m.simulate_scan(x,y,t/100.0)

#     cv2.imshow("map", map_image)
#     cv2.waitKey(1)
#     r.sleep()

cv2.destroyAllWindows()

"""
1 1 1 1 1 1 1 1 1 1 1 1 1
1 0 0 0 0 0 0 0 0 0 0 0 1
1 0 0 0 0 0 0 0 0 0 0 0 1
1 0 0 0 0 0 0 0 0 0 0 0 1
1 0 0 0 0 0 * 0 0 0 0 0 1
1 0 0 0 0 0 0 0 0 0 0 0 1
1 0 0 0 0 0 0 0 0 0 0 0 1
1 0 0 0 0 0 0 0 0 0 0 0 1
1 1 1 1 1 1 1 1 1 1 1 1 1

"""