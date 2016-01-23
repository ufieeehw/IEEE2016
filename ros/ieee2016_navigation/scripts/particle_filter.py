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
import matplotlib.pyplot as plt

PARTICLE_COUNT = 100

class ModelMap():
    def __init__(self):
        self.map = np.array([
            [ 1,  0,      0,          -1,   1e10,      0, 2.1336],
            [ 1,  0,  .4572,          -1,   1e10,   .762,  1.143],
            [ 1,  0,   .508,          -1,   1e10,      0,  .3048],
            [ 1,  0,   .762,          -1,   1e10,      0,  .3048],
            [ 1,  0, 1.8669,          -1,   1e10,  .8382, 2.1336],
            [ 1,  0, 2.1336,          -1,   1e10,      0,  .8382],
            [ 0,  1,      0,           0, 2.1336,     -1,   1e10],
            [ 0,  1,  .8382,      1.8669, 2.1336,     -1,   1e10],
            [ 0,  1,  .3048,        .508,   .762,     -1,   1e10],
            [ 0,  1, 2.1336,           0, 1.8669,     -1,   1e10],
        ])

        self.pub_map_scan = rospy.Publisher("map_scan", LaserScan, queue_size=2)

    def simulate_scan(self, *point):
        #print point
        angle_increment = .005 #rads/index
        min_angle = -3.14159274101 #rads
        max_angle = 3.14159274101
        max_range = 5.0 #m
        min_range = 0.00999999977648
        ranges = np.zeros(int((max_angle-min_angle)/angle_increment))
        
        index_count = len(ranges)
        #print index_count

        # Only ranges at these indexs will be generated
        # Pick ranges around where the LIDAR scans actually are (-90,0,90) degrees
        forty_deg_index = int(math.radians(10)/angle_increment)
        ranges_to_compare = np.zeros(0)
        step = 8
        ranges_to_compare = np.append( ranges_to_compare,            
            np.arange(index_count/4 - forty_deg_index, index_count/4 + forty_deg_index, step=step))
        ranges_to_compare = np.append( ranges_to_compare,            
            np.arange(index_count/2 - forty_deg_index, index_count/2 + forty_deg_index, step=step))
        ranges_to_compare = np.append( ranges_to_compare,            
            np.arange(3*index_count/4 - forty_deg_index, 3*index_count/4 + forty_deg_index, step=step))

        ranges_to_compare = np.arange(2000,step=8)

        for t in range(index_count):
            if t not in ranges_to_compare:
                #print i,"Nope"
                continue

            theta = min_angle + t*angle_increment + point[2]
            line = self.generate_line(point[:2],theta)
            #print theta,line
            # Test each wall to see if we intersect it
            hits = np.array([[0,0]])
            for w in self.map:
                #Just for displaying
                if w[0] == 0:
                    plt.axhline(w[2],xmin=w[3]/3,xmax=w[4]/3)
                #     #print "min",w[3],"max",w[4]
                if w[0] == 1:
                    plt.axvline(w[2],ymin=w[5]/3,ymax=w[6]/3)
                #     #print "min",w[5],"max",w[6]
                
                #print w[:3]
                matrix_to_solve = np.vstack((line,w[:3]))
                #print "m",matrix_to_solve
                coeff = matrix_to_solve[:,:2]
                #print "c",coeff
                dependent = matrix_to_solve[:,2]
                #print "d",dependent
                try:
                    # Calculate intersections
                    intersection = np.linalg.solve(coeff,dependent)
                    #print "i",intersection
                    # Remove intersections out of the domain of each line
                    xmin,xmax,ymin,ymax = w[3:]
                    if intersection[0] > xmax or intersection[0] < xmin or\
                       intersection[1] > ymax or intersection[1] < ymin: 
                        #print "line"
                        continue

                    # Remove intersections on the wrong side of the line
                    if (abs(theta) < 1.5708 and intersection[0] < point[0]) or\
                       (abs(theta) > 1.5708 and intersection[0] > point[0]) or\
                       (    theta  > 0      and intersection[1] < point[1]) or\
                       (    theta  < 0      and intersection[1] > point[1]):
                        continue

                    hits = np.concatenate((hits, [intersection]), axis=0)

                except:
                    # This means the line was parallel, so who cares about it
                    print "Error"
                    continue

            #All points found, now find the closest
            if len(hits) == 1: continue
            if len(hits) == 2: 
                ranges[t] = math.sqrt(math.pow((abs(point[0]-hits[1][0])),2) +
                                      math.pow((abs(point[1]-hits[1][1])),2))
                plt.plot([hits[1][0]],[hits[1][1]],'ro')
                continue

            dists = []
            for i in hits[1:]:
                dists.append(math.sqrt(math.pow((abs(point[0]-i[0])),2) +
                                       math.pow((abs(point[1]-i[1])),2)))
            plt.plot([hits[dists.index(min(dists))+1][0]],[hits[dists.index(min(dists))+1][1]],'ro')
            ranges[t] = min(dists)

        self.pub_map_scan.publish(LaserScan(    
                header=Header(
                stamp = rospy.Time.now(),
                frame_id = "odom",
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
        plt.plot([point[0]],[point[1]],'bo')
        # x = np.arange(4)
        # #x y n
        # formula = "(" + str(-line[0]) + "*x +" + str(line[2]) + ")"#/" + str(line[1])
        # y = eval(formula)
        # plt.plot(x,y)
        plt.axis([0, 3, 0, 3])
        plt.show()

    def generate_line(self,point,theta):
        # Returns the matrix row of a line passing through the point with theta rotation
        return np.array([-math.tan(theta)*point[0]/point[1], 1, point[1]-(math.tan(theta)*point[0])])



class Map():
    def __init__(self, map_name):
        global PARTICLE_COUNT
        # load map from bmp file in map/ folder
        self.map = cv2.imread(os.path.join(os.path.dirname(__file__), 'map/' + map_name + '.bmp'),0).astype(np.uint8)
        
        # Only for visulization
        # self.pub_map_scan = rospy.Publisher("map_scan", LaserScan, queue_size=2)
        # self.pub_list = []
        # for i in range(PARTICLE_COUNT):
        #     name = "sim_scan" + str(i)
        #     self.pub_list.append(rospy.Publisher(name, LaserScan, queue_size=2))

        # Load some meta data about the map
        with open(os.path.join(os.path.dirname(__file__), 'map/' + map_name + '.txt')) as f:
            meta_data = f.read().splitlines()

        self.cell_resolution = float(meta_data[1]) #m/px

        self.width = int(meta_data[2])
        self.height = int(meta_data[3])
        self.origin = [self.width/2,self.height/2]

    def simulate_scan_iterative_line_method(self, point, heading, name="map"):
        # Point should be y,x,heading (m,m,rads)
        # 0,0,0 is the middle of the map pointed right
        # Convert m points into px
        m_point = point,heading
        #print m_point

        point = np.array(point)/self.cell_resolution
        point += np.array([self.origin[0],self.origin[1]])

        ret,thresh = cv2.threshold(self.map,127,255,cv2.THRESH_BINARY)

        # Parameters of simulated laserscan (comes from comb lidar scan)
        angle_increment = .005 #rads/index
        min_angle = -3.14159274101 #rads
        max_angle = 3.14159274101
        max_range = 5.0 #m
        min_range = 0.00999999977648
        ranges = np.zeros(int((max_angle-min_angle)/angle_increment))
        
        index_count = len(ranges)
        #print index_count

        # Only ranges at these indexs will be generated
        # Pick ranges around where the LIDAR scans actually are (-90,0,90) degrees
        forty_deg_index = int(math.radians(10)/angle_increment)
        #print forty_deg_index
        ranges_to_compare = np.zeros(0)
        step = 8
        #print np.arange(index_count/4 - forty_deg_index, index_count/4 + forty_deg_index)
        ranges_to_compare = np.append( ranges_to_compare,            
            np.arange(index_count/4 - forty_deg_index, index_count/4 + forty_deg_index, step=step))
        ranges_to_compare = np.append( ranges_to_compare,            
            np.arange(index_count/2 - forty_deg_index, index_count/2 + forty_deg_index, step=step))
        ranges_to_compare = np.append( ranges_to_compare,            
            np.arange(3*index_count/4 - forty_deg_index, 3*index_count/4 + forty_deg_index, step=step))

        if name == "map": ranges_to_compare = np.arange(2000,step=8)
        #print ranges_to_compare
        # Temp image to store simulated laserscan results
        #lines = np.zeros(thresh.shape)
        for i in range(int((max_angle-min_angle)/angle_increment)):
            if i not in ranges_to_compare:
                #print i,"Nope"
                continue

            # Draw a test point from 'point' location out every 'angle_increment' rads from min_range to max_range
            # Once this test point intersects a wall in the map, save the data and create a LaserScan message with the data
            theta = min_angle + i*angle_increment + heading

            # Make test points along lines from origin and test if there is a wall at each point along the line
            # Only check for points between max and min range
            for r in range(int(min_range/self.cell_resolution),
                           int(max_range/self.cell_resolution)):
                x = point[0] + int(r * np.cos(theta))
                y = point[1] + int(r * np.sin(theta))
                
                # If the point is off the map, break
                if x >= self.width or y >= self.height: break
                if x <= 0 or y <= 0: break
                
                #lines[x,y] = 255
                # If the test point is a wall, add the dist to the list and break
                if thresh[self.height-y,x] == 255:
                    dist = math.sqrt((abs(point[0]-x))**2 + (abs(point[1]-y))**2)*self.cell_resolution
                    ranges[i] = dist
                    break
                #cv2.imshow("lines",lines)
                #cv2.waitKey(1)

            #cv2.imshow("thresh",thresh)
            #cv2.waitKey(1)
        #If we want to publish the scan, thats an option
        # br = tf.TransformBroadcaster()
        # if name=="map": 
        #     print "m"
        #     self.pub_map_scan.publish(LaserScan(    
        #         header=Header(
        #             stamp = rospy.Time.now(),
        #             frame_id = "odom",
        #             ),
        #         angle_min=min_angle,
        #         angle_max=max_angle,
        #         angle_increment=angle_increment,
        #         time_increment=0,
        #         scan_time=0,
        #         range_min=min_range,
        #         range_max=max_range,
        #         ranges=ranges.tolist(),
        #         intensities=[],
        #         )
        #     )
        # else:
        #     frame_name = "p"+str(name)
        #     br.sendTransform((m_point[0][0], m_point[0][1], 0),
        #             tf.transformations.quaternion_from_euler(0, 0, m_point[1]),
        #             rospy.Time.now(),
        #             frame_name,
        #             "odom")
        #     print frame_name
        #     self.pub_list[name].publish(LaserScan(    
        #         header=Header(
        #             stamp = rospy.Time.now(),
        #             frame_id = frame_name,
        #             ),
        #         angle_min=min_angle,
        #         angle_max=max_angle,
        #         angle_increment=angle_increment,
        #         time_increment=0,
        #         scan_time=0,
        #         range_min=min_range,
        #         range_max=max_range,
        #         ranges=ranges.tolist(),
        #         intensities=[],
        #         )
        #     )



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
    def __init__(self, x, y, heading, w = 0):
        # postition and rotation of particle and the weight of the particle (initally 0)
        self.x = x
        self.y = y
        self.heading = heading
        self.w = w

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
        self.test_points_pub = rospy.Publisher('test_points', PoseArray, queue_size=2)
        self.odom_sub = rospy.Subscriber('/robot/odometry/filtered', Odometry, self.got_odom)
        self.pose_est_pub = rospy.Publisher('pose_est', PoseStamped, queue_size=2)

        #self.m = Map("ieee_small")
        self.m = ModelMap()
        #self.m.simulate_scan_gradient_vector_method((0,0),0,0)
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

            self.particles.append(Particle(x,y,heading))

        self.hz_counter = 0
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.hz_counter = time.time()
            self.m.simulate_scan(1.22,1.22,1.57)
            print 1.0/(time.time()-self.hz_counter)
            r.sleep()
        # self.hz_counter = time.time()
        # #for i in range(p_count):
        # self.m.simulate_scan(1.22,1.22,0)
        # print 1.0/(time.time()-self.hz_counter)


    def run_filter(self):
        # This is where the filter does its work

        # Check our updated position from the last run of the filter, if it is 0 or close to it, then break and dont worry about running the filter
        # tolerance = 1e-4
        # if abs(self.pose_update[0]) < tolerance and\
        #    abs(self.pose_update[1]) < tolerance and\
        #    abs(self.pose_update[2]) < tolerance: return
        
        #print self.pose_update

        for i,p in enumerate(self.particles):
            p.update_pos(self.pose_update)
            particle_scan = self.m.simulate_scan(p.x,p.y,p.heading)
            #self.m.simulate_scan((0,0),0,"map")

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

rospy.init_node('particle_filter', anonymous=True)
f = Filter(PARTICLE_COUNT, (0,0), .5, (.1,-.1))
#rospy.spin()

# m = Map("ieee_small")
# map_image = m.map

# # Create a black image, a window
# cv2.namedWindow('bars')

# def nothing(x):
#     pass
# # create trackbars for color change
# cv2.createTrackbar('x','bars',0,150,nothing)
# cv2.createTrackbar('y','bars',0,150,nothing)
# cv2.createTrackbar('theta','bars',0,628,nothing)

# r = rospy.Rate(10)
# while not rospy.is_shutdown():
#     x = cv2.getTrackbarPos('x','bars')
#     y = cv2.getTrackbarPos('y','bars')
#     t = cv2.getTrackbarPos('theta','bars')
    
#     image = map_image
#     cv2.circle(image,(x+150,y+150),2,100,-1)

#     m.simulate_scan((x,y,t/100.0))

#     cv2.imshow("map", map_image)
#     cv2.waitKey(1)
#     r.sleep()

cv2.destroyAllWindows()



