#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
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
        #lines = np.zeros(thresh.shape)
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
                
                # If the point is off the map, break
                if x >= self.width or y >= self.height: break
                if x <= 0 or y <= 0: break

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
        # If we want to publish the scan, thats an option
        # self.pub.publish(LaserScan(    
        #     header=Header(
        #         stamp = rospy.Time.now(),
        #         frame_id = "map",
        #         ),
        #     angle_min=min_angle,
        #     angle_max=max_angle,
        #     angle_increment=angle_increment,
        #     time_increment=0,
        #     scan_time=0,
        #     range_min=min_range,
        #     range_max=max_range,
        #     ranges=ranges.tolist(),
        #     intensities=[],
        #     )
        # )
'''
middle
header: 
  seq: 677
  stamp: 
    secs: 1453340774
    nsecs: 453263500
  frame_id: laser_middle
angle_min: -1.57079637051
angle_max: 1.56466042995
angle_increment: 0.00613592332229
time_increment: 9.76562514552e-05
scan_time: 0.10000000149
range_min: 0.019999999553
range_max: 5.59999990463

combo
header: 
  seq: 80
  stamp: 
    secs: 1453341073
    nsecs: 716155052
  frame_id: laser_comb
angle_min: -3.14159274101
angle_max: 3.14159274101
angle_increment: 0.00300000002608
time_increment: 0.0
scan_time: 0.0
range_min: 0.00999999977648
range_max: 5.0
'''

class Particle():
    def __init__(self, parent_map, x, y, heading, w = 0):
        # postition and rotation of particle and the weight of the particle (initally 0)
        self.x = x
        self.y = y
        self.heading = heading
        self.w = w

        # The map to which the particle belongs
        self.parent_map = parent_map

    def update_weight(self, est_distance):
        # Found this method of calculating error in distance
        # Values close to 1 are closer more accurate

        sigma2 = 0.9 ** 2
        error = est_distance - measured_distance
        self.w = math.e ** -(error ** 2 / (2 * sigma2))

    def update_pos(self,(dx,dy,dhead), w = 0):
        self.x += dx
        self.y += dy
        self.heading += dhead
        self.w = w
        #print "updated"

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
        self.odom_sub = rospy.Subscriber('/robot/odometry/filtered', Odometry, self.got_odom)
        self.pose_est_pub = rospy.Publisher('pose_est', PoseStamped, queue_size=2)

        self.m = Map("ieee_small")

        # We start at arbitrary point 0,0,0
        self.pose = np.array([0,0,0], np.float64)
        self.pose_update = np.array([0,0,0], np.float64)

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

            self.particles.append(Particle(self.m,x,y,heading))

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.run_filter()
            r.sleep()

    def run_filter(self):
        # This is where the filter does its work

        # Check our updated position from the last run of the filter, if it is 0 or close to it, then break and dont worry about running the filter
        tolerance = 1e-4
        if abs(self.pose_update[0]) < tolerance and\
           abs(self.pose_update[1]) < tolerance and\
           abs(self.pose_update[2]) < tolerance: return
        
        print self.pose_update

        for p in self.particles:
            p.update_pos(self.pose_update)
        self.publish_particle_array()

        # Reset the pose update so that the next run will contain the pose update from this point
        self.pose_update = np.array([0,0,0], np.float64)


    def got_odom(self,msg):
        # Update current pose based on odom data, note that this is only an estimation of Shia's position
        # The twist is a measure of velocity from the previous state

        vehicle_twist = msg.twist.twist
        incoming_msg_freq = 100.0 #hz

        # This accounts for Shia's rotation - if he is pointed at a 45 degree angle and moves straight forward (which is what the twist message will say),
        # he is not moving directly along the x axis, he is moving at an offset angle.
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        rot_mat = np.matrix([
            [c,     -s],
            [s,      c],
        ], dtype=np.float32)
        # Then we add the x or y translation that we move, rounding down if it's super small
        x, y = np.dot(rot_mat, [vehicle_twist.linear.x/incoming_msg_freq, vehicle_twist.linear.y/incoming_msg_freq]).A1
        tolerance = 1e-7
        if abs(x) < tolerance: x = 0
        if abs(y) < tolerance: y = 0
        if abs(vehicle_twist.angular.z) < tolerance: vehicle_twist.angular.z = 0
        
        # By summing these components, we get an integral - converting velocity to position
        self.pose_update += [x, y, vehicle_twist.angular.z/incoming_msg_freq]
        self.pose += [x, y, vehicle_twist.angular.z/incoming_msg_freq]

        #print "POSE UPDATED"

        q = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
        self.pose_est_pub.publish(
            PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="odom"
                ),
                pose=Pose(
                    position=Point(
                        x=self.pose[0],
                        y=self.pose[1],
                        z=0
                    ),
                    orientation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3],
                    )
                )
            )

        )

    def got_laserscan(self,msg):
        pass

    def publish_particle_array(self):
        pose_arr = []
        for p in self.particles:
            pose_arr.append(p.return_pose())
            
        self.test_points_pub.publish(PoseArray(
            header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="odom",
                ),
            poses=pose_arr,
            )
        )
        #print "PUBLISHED PARTICLES"

f = Filter(PARTICLE_COUNT, (0,0), .5, (.1,-.1))
rospy.spin()

# m = Map("ieee_small")
# map_image = m.map

# # Create a black image, a window
# cv2.namedWindow('bars')


# def nothing(x):
#     pass
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