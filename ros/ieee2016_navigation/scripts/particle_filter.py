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
import os

import pyopencl as cl
os.environ['PYOPENCL_COMPILER_OUTPUT'] = '1'

class GPUAccMap():
    def __init__(self):
        # Each line segment in the form: ax1, ay1, ax2, ay2, bx1, by1, bx2, by2, ...
        self.map = np.array([0, 0, 0, .784, 0, .784, .015, .784, .015, .784, .015, 1.158, 0, 1.158, .015, 1.158, 0, 1.158, 0, 2.153, .464, .784, .479, .784, .479, .784, .479, 1.158, .464, .784, .464, 1.158, .464, 1.158, .479, 1.158, 0, 0, .549, 0, .549, 0, .549, .317, .549, .317, .569, .317, .569, 0, .569, .317, .569, 0, .809, 0, .809, 0, .809, .317, .809, .317, .829, .317, .829, 0, .829, .317, .829, 0, 2.458, 0, 0, 2.153, 2.458, 2.153, 2.458, 0, 2.458, .907, 2.161, .907, 2.458, .907, 2.161, .907, 2.161, 1.178, 2.161, 1.178, 2.458, 1.178, 2.458, 1.178, 2.458, 1.181, 2.161, 1.181, 2.458, 1.181, 2.161, 1.181, 2.161, 1.452, 2.161, 1.452, 2.458, 1.452, 2.458, 1.452, 2.458, 1.482, 2.161, 1.482, 2.458, 1.482, 2.161, 1.482, 2.161, 1.753, 2.161, 1.753, 2.458, 1.753, 2.458, 1.753, 2.458, 1.783, 2.161, 1.783, 2.458, 1.783, 2.161, 1.783, 2.161, 2.054, 2.161, 2.054, 2.458, 2.054, 2.458, 2.054, 2.458, 2.153]).astype(np.float32)

        # LaserScan parameters
        self.angle_increment = .005 #rads/index
        self.min_angle = -3.14159274101 #rads
        self.max_angle = 3.14159274101
        self.max_range = 5.0 #m
        self.min_range = 0.00999999977648

        index_count = int((self.max_angle - self.min_angle)/self.angle_increment)

        # Set up pyopencl
        self.ctx = cl.Context([cl.get_platforms()[1].get_devices()[0]])
        self.queue = cl.CommandQueue(self.ctx)
        self.mf = cl.mem_flags

        # Load .cl program
        f = open(os.path.join(os.path.dirname(__file__), 'particle_filter.cl'), 'r')
        fstr = "".join(f.readlines())
        self.prg = cl.Program(self.ctx, fstr).build()

        # Only ranges at these indicies will be checked
        # Pick ranges around where the LIDAR scans actually are (-90,0,90) degrees
        deg_index = int(math.radians(30)/self.angle_increment)
        self.indicies_to_compare = np.array([], np.int32)
        step = 1
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(index_count/4 - deg_index, index_count/4 + deg_index, step=step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(index_count/2 - deg_index, index_count/2 + deg_index, step=step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(3*index_count/4 - deg_index, 3*index_count/4 + deg_index, step=step))

        # To generate weights, we need those indices to be pre-converted to radian angle measures 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)

        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)
        self.map_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.map)

    def generate_weights(self, particles, laser_scan):
        # Particles format: ax, ay, aheading, bx, by, bheading, ...

        weights = np.zeros(particles.size/3).astype(np.float32)
        weights_cl = cl.Buffer(self.ctx, self.mf.WRITE_ONLY, weights.nbytes)

        laser_scan_compare = laser_scan[self.indicies_to_compare].astype(np.float32)
        particles_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=particles.astype(np.float32))
        laserscan_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=laser_scan_compare)
        
        # Actually send to graphics processor
        self.prg.trace(self.queue, (particles.size/3,), None,  particles_cl, 
                                                                self.map_cl, 
                                                 np.uint32(self.map.size/4), 
                                                  self.angles_to_compare_cl, 
                                     np.uint32(self.angles_to_compare.size),
                                                               laserscan_cl, 
                                                                 weights_cl).wait()
            
        cl.enqueue_copy(self.queue, weights, weights_cl).wait()
        return weights

class GPUAccFilter():
    def __init__(self, center, radius, heading_range, m):
        # Pass the max number of particles, the center and radius of where inital particle generation will be (meters), and the range of heading values (min,max)
        # ROS Inits
        self.test_points_pub = rospy.Publisher('/test_points', PoseArray, queue_size=2)
        self.laser_scan_vis = rospy.Publisher('/test_points', PoseArray, queue_size=2)
        self.odom_sub = rospy.Subscriber('/robot/odom', Odometry, self.got_odom)
        #self.twist_sub = rospy.Subscriber('/test/twist', TwistStamped, self.got_twist)
        self.laser_scan_sub = rospy.Subscriber('/scan_comb', LaserScan, self.got_laserscan)
        self.pose_est_pub = rospy.Publisher('/pf_pose_est', PoseStamped, queue_size=2)
        self.br = tf.TransformBroadcaster()

        self.m = m
        self.INIT_PARTICLES = 512
        self.MAX_PARTICLES = 2048

        # We start at our esitmated starting position
        self.pose = np.array([.2,.2,1.57], np.float32)
        self.pose_update = np.array([0,0,0], np.float32) 

        self.pose_est = np.array([.2,.2,1.57], np.float32)

        # Generate random point in circle and add to array of particle coordinates
        self.particles = np.empty([1,3])
        self.gen_particles(self.INIT_PARTICLES, center, radius, heading_range)

        # Remove the first index since its not actually a particle
        self.particles = self.particles[1:]
        self.publish_particle_array()
        #print self.particles
        self.laser_scan = np.array([])

        # For keeping track of time
        self.prev_time = time.time()

        self.hz_counter = 0
        r = rospy.Rate(20) #hz
        start_time = time.time()
        while not rospy.is_shutdown():
            r.sleep()
            
            self.run_filter()
            print "HZ:",1.0/(time.time()-start_time)
            start_time = time.time()

    def gen_particles(self, number_of_particles, center, radius, heading_range):
        print "GENERATING PARTICLES:", number_of_particles

        # Generate random particles in a circue with 'center' and 'radius'
        # heading_range gives a variance to the theta of each particle
        theta = np.random.rand((number_of_particles)) * (2 * np.pi)
        r = np.random.rand((number_of_particles))*radius
        # Clip ranges values that are generated that are off the map
        x = np.clip(r * np.cos(theta) + center[0],0,2.2)
        y = np.clip(r * np.sin(theta) + center[1],0,2.2)
        heading = np.random.uniform(heading_range[0], heading_range[1],(number_of_particles))

        # Save new particles in the proper format
        self.particles = np.vstack((self.particles,np.vstack((x,y,heading)).T))

    def run_filter(self):
        # This is where the filter does its work

        # Check our updated position from the last run of the filter, if it is 0 or close to it, then break and dont worry about running the filter
        # tolerance = 1e-4
        # if abs(self.pose_update[0]) < tolerance and\
        #    abs(self.pose_update[1]) < tolerance and\
        #    abs(self.pose_update[2]) < tolerance: return
        while len(self.laser_scan) == 0:
            print "Waiting for scan."

        self.particles += self.pose_update
        
        # Reset the pose update so that the next run will contain the pose update from this point
        self.pose_update = np.array([0,0,0], np.float32)

        # If new particles need to be generated, do so with this radius from the last estimated position
        new_gen_radius = 1
        #if len(self.particles) == 0: self.gen_particles(self.INIT_PARTICLES, self.pose_est[:2],new_gen_radius,(self.pose_est[2]-1,self.pose_est[2]+1))

        #self.particles = temp_particles[1:]

        # weights holds particles weights. The more accurate a measurement was, the close to 1 it will be.
        weights_raw = self.m.generate_weights(self.particles,self.laser_scan)

        # # Remove low weights from particle and weights list
        weight_percentile = 90 #percent
        weights_indicies_to_keep = weights_raw > np.percentile(weights_raw,weight_percentile)
        weights = weights_raw[weights_indicies_to_keep]
        self.particles = self.particles[weights_indicies_to_keep]

        # if len(self.particles) == 0: self.gen_particles(self.INIT_PARTICLES, self.pose_est[:2],new_gen_radius,(self.pose_est[2]-1,self.pose_est[2]+1))


        # #Just for debugging ==================================
        print "WEIGHT PERCENTILE:", weight_percentile
        print "CUTOFF:", np.percentile(weights_raw,weight_percentile)
        print "PARTICLES REMOVED:", len(weights_raw)-len(weights) 
        print "PARTICLE COUNT:", len(self.particles)

    
        # Calculate pose esitmation before generating new particles
        new_x = np.mean(self.particles.T[0])
        new_y = np.mean(self.particles.T[1])
        new_head = np.mean(self.particles.T[2])

        # Update Pose
        self.update_pose((new_x,new_y,new_head))

        translation_vairance = .5  #m  #.1
        rotational_vairance = .6 #rads #.5

        self.gen_particles( self.MAX_PARTICLES - len(self.particles), 
                            self.pose_est[:2],
                            translation_vairance,
                            (self.pose_est[2]-rotational_vairance,self.pose_est[2]+rotational_vairance) )
        
        print 
    
    def update_real_pose(self,msg):
        roll,pitch,yaw = tf.transformations.euler_from_quaternion((msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w))
        self.pose = np.array([msg.pose.position.x,msg.pose.position.y,yaw])

    def update_pose(self,particle_avg):
        # Update pose and TF
        self.pose_est = np.array(particle_avg)

        #print self.pose
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose_est[2])
        self.pose_est_pub.publish(
            PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                ),
                pose=Pose(
                    position=Point(
                        x=self.pose_est[0],
                        y=self.pose_est[1],
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

        self.br.sendTransform((self.pose_est[0], self.pose_est[1], 0), q,
                 rospy.Time.now(),
                 "base_link",
                 "map")

    def got_twist(self,msg):
        # Just a temp method to test the filter
        vehicle_twist = msg.twist
        
        time_since_last_msg = self.prev_time - time.time() #seconds
        self.prev_time = time.time()
        incoming_msg_freq = 1.0#/time_since_last_msg

        # This accounts for Shia's rotation - if he is pointed at a 45 degree angle and moves straight forward (which is what the twist message will say),
        # he is not moving directly along the x axis, he is moving at an offset angle.
        c, s = np.cos(self.pose_est[2]), np.sin(self.pose_est[2])
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
        # self.pose_est_pub.publish(
        #     PoseStamped(
        #         header=Header(
        #             stamp=rospy.Time.now(),
        #             frame_id="map"
        #         ),
        #         pose=Pose(
        #             position=Point(
        #                 x=self.pose[0],
        #                 y=self.pose[1],
        #                 z=0
        #             ),
        #             orientation=Quaternion(
        #                 x=q[0],
        #                 y=q[1],
        #                 z=q[2],
        #                 w=q[3],
        #             )
        #         )
        #     )

        # )

    def got_laserscan(self,msg):
        self.laser_scan = np.array(msg.ranges)

    def publish_particle_array(self):
        pose_arr = []

        #print "PUBLISHING POSE ARRAY"
        for p in self.particles:
            if any(np.isnan(p)) or any(np.isinf(p)):
                print "INVAILD POINT DETECTED"
                continue
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
                    frame_id="map",
                ),
            poses=pose_arr,
            )
        )
        #print "PUBLISHED PARTICLES"


def add_noise(values,noise_size):
    # Adds noise to scan to some data
    return values + np.random.uniform(0,noise_size-noise_size/2.0,values.size)


rospy.init_node('particle_filter', anonymous=True)
m = GPUAccMap()
f = GPUAccFilter((.2,.2), .5, (1.3,1.8), m)

rospy.spin()



