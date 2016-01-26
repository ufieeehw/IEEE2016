#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import LaserScan
import tf

import numpy as np
import math
import time
import random
import matplotlib.pyplot as plt
from threading import Thread
import os

import pyopencl as cl


PARTICLE_COUNT = 100

class GPUAccMap():
    def __init__(self):
        # Each line segment in the form: ax1, ay1, ax2, ay2, bx1, by1, bx2, by2, ...
        self.map = np.array([      0,     0,     0,2.1336,
                               .4572,  .762, .4572, 1.143,
                                .508,     0,  .508, .3048,
                                .762,     0,  .762, .3048,
                              1.8669, .8382,1.8669,2.1336,
                              2.1336,     0,2.1336, .8382,
                                   0,     0,2.1336,     0,
                              1.8669, .8382,2.1336, .8382,
                                .508, .3048,  .762, .3048,
                                   0,2.1336,1.8669,2.1336]).astype(np.float32)
    
        # self.map = np.array([
        #         2,2,0,2,
        #         0,0,2,0,
        #         0,0,0,2,
        #         2,0,2,2
        #     ]).astype(np.float32)
        self.map_pub = rospy.Publisher("/test/map_scan", LaserScan, queue_size=1)

        # LaserScan parameters
        self.angle_increment = .005 #rads/index
        self.min_angle = -3.14159274101 #rads
        self.max_angle = 3.14159274101
        self.max_range = 5.0 #m
        self.min_range = 0.00999999977648

        index_count = int((self.max_angle - self.min_angle)/self.angle_increment)

        # Set up pyopencl
        self.ctx = cl.create_some_context()
        self.queue = cl.CommandQueue(self.ctx)
        self.mf = cl.mem_flags
        # Load .cl program
        f = open(os.path.join(os.path.dirname(__file__), 'particle_filter.cl'), 'r')
        fstr = "".join(f.readlines())
        self.prg = cl.Program(self.ctx, fstr).build()

        # Only ranges at these indicies will be checked
        # Pick ranges around where the LIDAR scans actually are (-90,0,90) degrees
        deg_index = int(math.radians(20)/self.angle_increment)
        self.indicies_to_compare = np.array([], np.int32)
        step = 1
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(index_count/4 - deg_index, index_count/4 + deg_index, step=step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(index_count/2 - deg_index, index_count/2 + deg_index, step=step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(3*index_count/4 - deg_index, 3*index_count/4 + deg_index, step=step))
        
        #self.indicies_to_compare = np.array([0,314,628,1255])

        # To generate weights, we need those indices to be pre-converted to radian angle measures 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)

        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)
        self.map_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.map)

    def generate_weights(self, particles, laser_scan):
        weights = np.zeros(self.indicies_to_compare.size).astype(np.float32)
        laser_scan_compare = laser_scan[self.indicies_to_compare]
        particles_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=particles)
        laserscan_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=laser_scan_compare)
        
        # Actually send to graphics processor
        self.prg.trace(self.queue, point.size/3, None, particles_cl, 
                                                        self.map_cl, 
                                         np.uint32(self.map.size/4), 
                                          self.angles_to_compare_cl, 
                             np.uint32(self.angles_to_compare.size),
                                                       laserscan_cl, 
                                                    self.weights_cl)


    def simulate_scan(self, point):
        # Not really weights, just a holder for the ranges
        weights = np.zeros(self.indicies_to_compare.size).astype(np.float32)
        self.weights_cl = cl.Buffer(self.ctx, self.mf.WRITE_ONLY, weights.nbytes)

        point_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=point)
        temp = np.zeros(self.indicies_to_compare.size).astype(np.float32)
        temp_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=temp)

        self.prg.trace(self.queue, (1,), None, point_cl, 
                                            self.map_cl, 
                             np.uint32(self.map.size/4), 
                              self.angles_to_compare_cl, 
                 np.uint32(self.angles_to_compare.size),
                                                temp_cl, 
                                        self.weights_cl)

        #self.weights_t = np.empty_like(self.weights)
        cl.enqueue_copy(self.queue, weights, self.weights_cl)
        #print self.angles_to_compare
        #print self.weights

        # Just for publishing laserscans
        self.ranges = np.zeros(index_count)
        ranges[self.indicies_to_compare] = weights

        self.map_pub.publish(LaserScan(    
            header=Header(
                stamp = rospy.Time.now(),
                frame_id = "base_link",
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
        self.pub_list = []
        self.br = tf.TransformBroadcaster()
        # for i in range(PARTICLE_COUNT):
        #     name = "sim_scan" + str(i)
        #     self.pub_list.append(rospy.Publisher(name, LaserScan, queue_size=2))
        self.map_pub = rospy.Publisher("/test/map_scan", LaserScan, queue_size=2)

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
        deg_index = int(math.radians(20)/self.angle_increment)
        self.ranges_to_compare = np.array([], np.int32)
        step = 2
        self.ranges_to_compare = np.append( self.ranges_to_compare,            
            np.arange(index_count/4 - deg_index, index_count/4 + deg_index, step=step))
        self.ranges_to_compare = np.append( self.ranges_to_compare,            
            np.arange(index_count/2 - deg_index, index_count/2 + deg_index, step=step))
        self.ranges_to_compare = np.append( self.ranges_to_compare,            
            np.arange(3*index_count/4 - deg_index, 3*index_count/4 + deg_index, step=step))

        self.real_ranges = np.array([], np.int32)
        self.real_ranges = np.append( self.real_ranges,            
            np.arange(index_count/4 - deg_index, index_count/4 + deg_index))
        self.real_ranges = np.append( self.real_ranges,            
            np.arange(index_count/2 - deg_index, index_count/2 + deg_index))
        self.real_ranges = np.append( self.real_ranges,            
            np.arange(3*index_count/4 - deg_index, 3*index_count/4 + deg_index))

        #print self.real_ranges
        #ranges_to_compare = np.arange(2000)
    def simulate_scan(self, point, heading, name):
        # Make sure the point is a numpy array
        point = np.array(point)

        if name == "real": self.ranges_to_compare = self.real_ranges

        for t in self.ranges_to_compare:
            theta = self.min_angle + t*self.angle_increment + heading

            ray_direction = np.array([math.cos(theta), math.sin(theta)])
            intersections = []
            for w in self.map:
                intersection_dist = self.find_intersection(point, ray_direction, w[0],w[1])
                if intersection_dist is not None:
                    # If the intersection distance is not in range don't worry about it
                    if intersection_dist > self.max_range or intersection_dist < self.min_range: continue

                    intersections.append(intersection_dist)

            #All intersection points found, now find the closest
            if len(intersections) > 0:
                self.ranges[t] = min(intersections)

        #self.ranges[self.real_ranges] = self.add_noise(self.ranges[self.real_ranges])

        #frame_name = "p"+str(name)
        self.br.sendTransform((point[0], point[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, heading),
                rospy.Time.now(),
                "base_link",
                "odom")
        #print frame_name
        self.map_pub.publish(LaserScan(    
            header=Header(
                stamp = rospy.Time.now(),
                frame_id = "base_link",
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

        return self.ranges[self.ranges_to_compare]
    
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

    def add_noise(self,ranges):
        # Adds noise to scan to simulate real scan
        noise_size = .03 #m
        return ranges + np.random.uniform(0,noise_size,ranges.size)

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

class GPUAccFilter(Thread):
    def __init__(self, p_count, center, radius, heading_range, m):
        # Pass the max number of particles, the center and radius of where inital particle generation will be (meters), and the range of heading values (min,max)
        # ROS Inits
        self.test_points_pub = rospy.Publisher('/test/test_points', PoseArray, queue_size=2)
        #self.odom_sub = rospy.Subscriber('/robot/odometry/filtered', Odometry, self.got_odom)
        self.twist_sub = rospy.Subscriber('/test/test_twist', TwistStamped, self.got_twist)
        self.laser_scan_sub = rospy.Subscriber('/test/map_scan', LaserScan, self.got_laserscan)
        self.pose_est_pub = rospy.Publisher('/test/pose_est', PoseStamped, queue_size=2)

        self.m = m

        # We start at arbitrary point 0,0,0
        self.pose = np.array([.2,.2,1.57], np.float64)
        self.pose_update = np.array([0,0,0], np.float64) 

        # Generate random point in circle and add to array of point coordinates
        self.particles = np.empty([1,3])
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

            self.particles = np.vstack((self.particles,[x,y,heading]))

        # Remove the first index since its not actually a particle
        self.particles = self.particles[1:]
        #print self.particles
        self.laser_scan = np.array([])


        # For keeping track of time
        self.prev_time = time.time()

        self.hz_counter = 0
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            r.sleep()
            self.hz_counter = time.time()
            self.run_filter()
            print 1.0/(time.time()-self.hz_counter)
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
        while len(self.laser_scan) == 0:
            print "No scan"

        #laser_scan = np.copy(self.laser_scan)[self.m.ranges_to_compare]
        
        self.particles += self.pose_update
        self.m.generate_weights(self.particles,self.laser_scan)

        # Reset the pose update so that the next run will contain the pose update from this point
        self.pose_update = np.array([0,0,0], np.float64)

        self.publish_particle_array()

    def got_twist(self,msg):
        # Just a temp method to test the filter
        vehicle_twist = msg.twist
        
        time_since_last_msg = self.prev_time - time.time() #seconds
        self.prev_time = time.time()
        incoming_msg_freq = 1.0#/time_since_last_msg

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

    def got_odom(self,msg):
        # Update current pose based on odom data, note that this is only an estimation of Shia's position
        # The twist is a measure of velocity from the previous state

        vehicle_twist = msg.twist.twist
        incoming_msg_freq = 100 #hz

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
        self.laser_scan = np.array(msg.ranges)

    def publish_particle_array(self):
        pose_arr = []
        for p in self.particles:
            q = tf.transformations.quaternion_from_euler(0, 0, p[2])
            pose = Pose(
                position=Point(
                        x=p[0],
                        y=p[1],
                        z=0
                    ),
                orientation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3],
                    )
            )
            pose_arr.append(pose)

        self.test_points_pub.publish(PoseArray(
            header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="odom",
                ),
            poses=pose_arr,
            )
        )
        #print "PUBLISHED PARTICLES"

class Filter(Thread):
    def __init__(self, p_count, center, radius, heading_range, m):
        # Pass the max number of particles, the center and radius of where inital particle generation will be (meters), and the range of heading values (min,max)
        # ROS Inits
        self.test_points_pub = rospy.Publisher('/test_points', PoseArray, queue_size=2)
        self.odom_sub = rospy.Subscriber('/robot/odometry/filtered', Odometry, self.got_odom)
        self.twist_sub = rospy.Subscriber('/test/test_twist', TwistStamped, self.got_twist)
        self.pose_est_pub = rospy.Publisher('pose_est', PoseStamped, queue_size=2)

        self.m = m

        # We start at arbitrary point 0,0,0
        self.pose = np.array([.2,.2,1.57], np.float64)
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

        self.laser_scan = np.array([])

        # For keeping track of time
        self.prev_time = time.time()

        self.hz_counter = 0
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            r.sleep()
            self.hz_counter = time.time()
            self.run_filter()
            print 1.0/(time.time()-self.hz_counter)
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
        while len(self.laser_scan) == 0:
            print "No scan"
        update = np.copy(self.pose_update)
        laser_scan = np.copy(self.laser_scan)[self.m.ranges_to_compare]
        # Reset the pose update so that the next run will contain the pose update from this point
        self.pose_update = np.array([0,0,0], np.float64)

        for i,p in enumerate(self.particles):
            p.update_pos(update)
            particle_scan = self.m.simulate_scan((p.x,p.y),p.heading,i)
            p.w = self.compare_with_scan(particle_scan,laser_scan)
            if p.w > .9: print i,
        self.publish_particle_array()
    
    def compare_with_scan(self,measured_scan,sim_scan):
        # Found this method online
        # More accurate values => 1, less accurate => 0
        sigma2 = .3 ** 2
        error = measured_scan - sim_scan
        g = np.mean(math.e ** -(error ** 2 / (2 * sigma2)))

        return g

    def got_twist(self,msg):
        # Just a temp method to test the filter
        vehicle_twist = msg.twist
        
        time_since_last_msg = self.prev_time - time.time() #seconds
        self.prev_time = time.time()
        incoming_msg_freq = 1.0#/time_since_last_msg

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

    def got_odom(self,msg):
        # Update current pose based on odom data, note that this is only an estimation of Shia's position
        # The twist is a measure of velocity from the previous state

        vehicle_twist = msg.twist.twist
        incoming_msg_freq = 100 #hz

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
        self.laser_scan = np.array(msg.ranges)

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

class Test(Thread):
    def __init__(self,m,m2):
        self.odom_sub = rospy.Subscriber('/spacenav/twist', Twist, self.got_twist)
        self.est_pose_pub = rospy.Publisher('/test/pose', PoseStamped, queue_size=2)
        self.twist_pub = rospy.Publisher('/test/test_twist', TwistStamped, queue_size=2)

        self.pose = np.array([1,1,1.57])

        self.m = m
        self.m2 = m2

        self.speed_multiplier = .01

        self.br = tf.TransformBroadcaster()

        l = Thread(target=self.pub_laserscan)
        l.start()


    def got_twist(self,msg):
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        rot_mat = np.matrix([
            [c,     -s],
            [s,      c],
        ], dtype=np.float32)
        x, y = np.dot(rot_mat, [msg.linear.x, msg.linear.y]).A1
        
        self.pose += np.array([x,y,msg.angular.z])*self.speed_multiplier

        self.publish_pose(msg)

    def pub_laserscan(self):
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            print "Pose",self.pose
            self.br.sendTransform((self.pose[0], self.pose[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, self.pose[2]),
                rospy.Time.now(),
                "base_link",
                "odom")

            #self.m.simulate_scan(np.array([self.pose[0],self.pose[1],self.pose[2]]).astype(np.float32))
            #self.m.simulate_scan(np.array([self.pose[0],self.pose[1],self.pose[2]]).astype(np.float32))
            self.m2.simulate_scan((self.pose[0],self.pose[1]), self.pose[2], "real")
            r.sleep()

    def publish_pose(self,twist):
        #print "Publishing Pose",self.pose

        q = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
        p = PoseStamped(
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
        self.est_pose_pub.publish(p)

        t = TwistStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="odom"
            ),
            twist=Twist(
                linear=Vector3(
                    x=twist.linear.x*self.speed_multiplier,
                    y=twist.linear.y*self.speed_multiplier,
                    z=0.0
                ),
                angular=Vector3(
                    x=0.0,
                    y=0.0,
                    z=twist.angular.z*self.speed_multiplier
                )
            )
        )
        self.twist_pub.publish(t)



rospy.init_node('particle_filter', anonymous=True)
m = GPUAccMap()
m2 = ModelMap()
t = Test(m,m2)
f = GPUAccFilter(PARTICLE_COUNT, (1,1), .15, (1.3,1.8), m)


#f = Filter(PARTICLE_COUNT, (.21,.23), .15, (1.3,1.8), m,m2)

t.start()

rospy.spin()



