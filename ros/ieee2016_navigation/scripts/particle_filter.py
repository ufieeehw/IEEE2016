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

PARTICLE_COUNT = 10

class ModelMap():
    def __init__(self):
        global PARTICLE_COUNT
        self.map = np.array([
            [[     0,     0],[     0,2.1336]],
            [[ .4572,  .762],[ .4572, 1.143]],
            [[  .508,     0],[  .508, .3048]],
            [[  .762,     0],[  .762, .3048]],
            [[1.8669, .8382],[1.8669,2.1336]],
            [[2.1336,     0],[2.1336, .8382]],
            [[     0,     0],[2.1336,    0,]],
            [[1.8669, .8382],[2.1336, .8382]],
            [[  .508, .3048],[  .762, .3048]],
            [[     0,2.1336],[1.8669,2.1336]]
        ])

        #Only for visulization
        self.pub_map_scan = rospy.Publisher("map_scan", LaserScan, queue_size=2)
        self.pub_list = []
        self.br = tf.TransformBroadcaster()
        for i in range(PARTICLE_COUNT):
            name = "sim_scan" + str(i)
            self.pub_list.append(rospy.Publisher(name, LaserScan, queue_size=2))

        #print point
        self.angle_increment = .005 #rads/index
        self.min_angle = -3.14159274101 #rads
        self.max_angle = 3.14159274101
        self.max_range = 5.0 #m
        self.min_range = 0.00999999977648
        self.ranges = np.zeros(int((self.max_angle-self.min_angle)/self.angle_increment))
        
        index_count = len(self.ranges)

        # Only ranges at these indexs will be generated
        # Pick ranges around where the LIDAR scans actually are (-90,0,90) degrees
        forty_deg_index = int(math.radians(10)/self.angle_increment)
        self.ranges_to_compare = np.zeros(0)
        step = 4
        self.ranges_to_compare = np.append( self.ranges_to_compare,            
            np.arange(index_count/4 - forty_deg_index, index_count/4 + forty_deg_index, step=step))
        self.ranges_to_compare = np.append( self.ranges_to_compare,            
            np.arange(index_count/2 - forty_deg_index, index_count/2 + forty_deg_index, step=step))
        self.ranges_to_compare = np.append( self.ranges_to_compare,            
            np.arange(3*index_count/4 - forty_deg_index, 3*index_count/4 + forty_deg_index, step=step))

        #ranges_to_compare = np.arange(2000)
    def simulate_scan(self, point, heading,name):
        # Make sure the point is a numpy array
        point = np.array(point)

        for t in self.ranges_to_compare:
            theta = self.min_angle + t*self.angle_increment + heading

            ray_direction = np.array([math.cos(theta), math.sin(theta)])
            intersections = []
            for w in self.map:
                intersection_dist = self.find_intersection(point, ray_direction, w[0],w[1])
                if intersection_dist is not None:
                    intersections.append(intersection_dist)

            #All intersection points found, now find the closest
            if len(intersections) > 0:
                self.ranges[t] = min(intersections)

        frame_name = "p"+str(name)
        self.br.sendTransform((point[0], point[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, heading),
                rospy.Time.now(),
                frame_name,
                "odom")
        print frame_name
        self.pub_list[name].publish(LaserScan(    
            header=Header(
                stamp = rospy.Time.now(),
                frame_id = frame_name,
                ),
            angle_min=self.min_angle,
            angle_max=self.max_angle,
            angle_increment=self.angle_increment,
            time_increment=0,
            scan_time=0,
            range_min=self.min_range,
            range_max=self.max_range,
            ranges=self.ranges.tolist(),
            intensities=[],
            )
        )
        #plt.plot([point[0]],[point[1]],'bo')
        # x = np.arange(4)
        # #x y n
        # formula = "(" + str(-line[0]) + "*x +" + str(line[2]) + ")"#/" + str(line[1])
        # y = eval(formula)
        # plt.plot(x,y)
        #plt.axis([0, 3, 0, 3])
        #plt.show()
    
    def find_intersection(self, ray_origin, ray_direction, point1, point2):
        # Ray-Line Segment Intersection Test in 2D
        # http://bit.ly/1CoxdrG
        v1 = ray_origin - point1
        v2 = point2 - point1
        v3 = np.array([-ray_direction[1], ray_direction[0]])

        v2_dot_v3 = np.dot(v2, v3)
        if v2_dot_v3 == 0:
            return None

        t1 = np.cross(v2, v1) / v2_dot_v3
        t2 = np.dot(v1, v3) / v2_dot_v3
        
        if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
            return t1
        return None

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
            self.run_filter()
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
            particle_scan = self.m.simulate_scan((p.x,p.y),p.heading,i)
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
f = Filter(PARTICLE_COUNT, (.2,.2), .15, (1.54,1.61))

cv2.destroyAllWindows()



