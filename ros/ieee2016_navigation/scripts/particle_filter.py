#!/usr/bin/python
import rospy
from std_msgs.msg import Header, Float32
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, \
        PoseWithCovarianceStamped, PoseWithCovariance, Twist, TwistStamped, Vector3, TwistWithCovarianceStamped
from sensor_msgs.msg import LaserScan

from ieee2016_msgs.srv import ResetOdom
from ieee2016_msgs.srv import LidarSelector
from ieee2016_msgs.msg import StartNavigation

import tf

import numpy as np
import math
import time
import random
import os
import struct

import pyopencl as cl
os.environ['PYOPENCL_COMPILER_OUTPUT'] = '1'

class LidarCorrector():
    '''
    Stores some information sent to the GPU to do some correction for bad data.
    
    We could potentially modify and use this in other places, but for right now it only is used for sending
    parameters to the GPU.

    Pass in the parameters of the fused scan, then add each lidar - indicating which direction it's pointing.
    '''
    def __init__(self):
        self.lidar_list = []

    def add_lidar(self, calibration_coeff, min_distance):
        lidar = [float(calibration_coeff[0]),float(calibration_coeff[1]),
                 float(calibration_coeff[2]),float(calibration_coeff[3]),float(min_distance)]

        self.lidar_list.append(lidar)
    
    def pack(self):
        packed = []
        #s = struct.Struct('ii ffff f xxxx')
        s = struct.Struct('ffff f xxxxxxxxxxxx')
        for lidar in self.lidar_list:
            packed.append(s.pack(*lidar))
            print packed
        packed = b''.join(packed)
        #print binascii.hexlify(packed)
        #print len(packed)
        return packed


class GPUAccMap():
    def __init__(self,map_version):
        # Each line segment in the form: ax1, ay1, ax2, ay2, color_code, bx1, by1, bx2, by2, color_code, ...
        # Color code: 1 for dark (black), 0 for anything else
        self.map = np.array(map_version).astype(np.float32)

        # LaserScan parameters
        self.angle_increment = .007 #rads/index
        self.min_angle = -3.14159274101 #rads
        self.max_angle = 3.14159274101
        self.max_range = 5.0 #m
        self.min_range = 0.1

        self.index_count = int((self.max_angle - self.min_angle)/self.angle_increment)

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
        fov = math.radians(40)
        self.deg_index = int(fov/self.angle_increment)
        self.indicies_to_compare = np.array([], np.int32)
        self.angle_to_lidar = np.array([], np.uint32)
        self.step = 1

        # Define the lidar Corrector, and init the lidars - order is important!
        # The addition to the min_range is to account for the fact the lidars are offset from base_link
        lidar = LidarCorrector()
        lidar.add_lidar([.052133885,.0298194301,-.0783299411,.0249704984],self.min_range+.1)         # Back
        lidar.add_lidar([-.0102461432,.0205318341,.0208797008,-.00183668788],self.min_range+.125)    # Left
        lidar.add_lidar([.0451106369,-.1020802842,.1752494951,-.0700421761],self.min_range+.125)     # Front
        lidar.add_lidar([.0118628147,.1439880093,-.1344849981,.0298791165],self.min_range+.1)        # Right
        packed_lidar = lidar.pack()

        # The serivce to determine which LIDAR to navigate with
        # We also add the lidar placement numbers here. 0 is the rear lidar then go clockwise around so 3 is the right lidar
        #s = rospy.Service('/robot/navigation/select_lidar', LidarSelector, self.change_indicies_to_compare)
        # Right ----------------
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(self.index_count/4 - self.deg_index, self.index_count/4 + self.deg_index, step=self.step))
        self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,3))
        # Front ----------------
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(self.index_count/2 - self.deg_index, self.index_count/2 + self.deg_index, step=self.step))
        self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,2))
        # Left -----------------
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(3*self.index_count/4 - self.deg_index, 3*self.index_count/4 + self.deg_index, step=self.step))
        self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,1))
        # Back ----------------
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(0, self.deg_index, step=self.step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(self.index_count - self.deg_index, self.index_count, step=self.step))
        self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,0))

        # To generate weights, we need those indices to be pre-converted to radian angle measures 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)

        # Set up opencl buffers (where to put data to transfer to the gpu and back)
        self.map_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.map)
        self.lidar_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=packed_lidar)
        self.angle_to_lidar_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angle_to_lidar.astype(np.int32))
        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)

        print "Map Init complete."
    def generate_weights(self, particles, laser_scan):
        # Particles format: ax, ay, aheading, bx, by, bheading, ...
        
        temp = np.zeros(self.angles_to_compare.size*particles.size/3).astype(np.float32)
        temp_cl = cl.Buffer(self.ctx, self.mf.WRITE_ONLY, temp.nbytes)

        weights = np.zeros(particles.size/3).astype(np.float32)
        weights_cl = cl.Buffer(self.ctx, self.mf.WRITE_ONLY, weights.nbytes)

        laser_scan_compare = laser_scan[self.indicies_to_compare].astype(np.float32)
        particles_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=particles.astype(np.float32))
        laserscan_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=laser_scan_compare)
        
        # Actually send to graphics processor
        self.prg.trace(self.queue, (particles.size/3,), None,  particles_cl, 
                                                                self.map_cl, 
                                                              self.lidar_cl,
                                                     self.angle_to_lidar_cl,
                                                 np.uint32(self.map.size/5), 
                                                  self.angles_to_compare_cl, 
                                     np.uint32(self.angles_to_compare.size),
                                                               laserscan_cl, 
                                                                 weights_cl).wait()


        cl.enqueue_copy(self.queue, weights, weights_cl).wait()
        #cl.enqueue_copy(self.queue, temp, temp_cl).wait()

        #print np.where(temp > .5)
        
        return weights

    def change_indicies_to_compare(self,srv):
        lidars = srv.lidar_names

        # This look like a big mess, and it is.
        # We add the indicies of each LIDAR's FOV to the master list depending if it was selected or not
        self.indicies_to_compare = np.array([])
        if "right" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count/4 - self.deg_index, self.index_count/4 + self.deg_index, step=self.step) )
            self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,3))
        if "front" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count/2 - self.deg_index, self.index_count/2 + self.deg_index, step=self.step) )
            self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,2))
        if "left" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(3*self.index_count/4 - self.deg_index, 3*self.index_count/4 + self.deg_index, step=self.step) )
            self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,1))
        if "back" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(0, self.deg_index, step=self.step)).astype(np.int)
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count - self.deg_index, self.index_count, step=self.step))
            self.angle_to_lidar = np.append(self.angle_to_lidar, np.full(self.deg_index*2/self.step,0))
            
        # Update other necessary parameters 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)
        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)
        self.angle_to_lidar_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angle_to_lidar.astype(np.int32))
        print "Change",self.indicies_to_compare
        return True

class GPUAccFilter():
    def __init__(self, center, radius, heading_range, m):
        # Pass the max number of particles, the center and radius of where inital particle generation will be (meters), and the range of heading values (min,max)

        self.test_points_pub = rospy.Publisher('/test_points', PoseArray, queue_size=2)
        self.pose_est_pub = rospy.Publisher('/robot/navigation/pf_pose_vis', PoseStamped, queue_size=2)
        self.p_c_s_est_pub = rospy.Publisher('/robot/navigation/pf_pose', PoseWithCovarianceStamped, queue_size=10)

        self.odom_sub = rospy.Subscriber('/robot/navigation/odom_twist', TwistWithCovarianceStamped, self.got_twist)
        self.laser_scan_sub = rospy.Subscriber('/robot/navigation/lidar/scan_fused', LaserScan, self.got_laserscan)

        self.br = tf.TransformBroadcaster()

        self.m = m
        self.INIT_PARTICLES = 2048
        self.MAX_PARTICLES = 2048

        # We start at our esitmated starting position
        self.pose = np.array([center[0],center[1],sum(heading_range)/2.0], np.float32)
        self.pose_update = np.array([0,0,0], np.float32) 

        self.pose_est = np.array([center[0],center[1],sum(heading_range)/2.0], np.float32)

        # Generate random point in circle and add to array of particle coordinates
        self.particles = np.empty([1,3])
        self.gen_particles(self.INIT_PARTICLES, center, radius, heading_range)

        # Remove the first index since its not actually a particle
        self.particles = self.particles[1:]
        self.publish_particle_array()
        #print self.particles
        self.laser_scan = np.array([])

        # Begin running the filter
        print "Running Filter."
        #self.prev_time = time.time()
        self.run_filter()

    def got_pose_estimate(self, msg):
        print msg
        new_yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])[2]
        new_pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,new_yaw])
        self.pose_update = np.array([0,0,0])
        self.pose = np.array(new_pose)

        # Make new particles around that spot
        self.particles = np.zeros([1,3])
        self.gen_particles(self.INIT_PARTICLES, new_pose[:2], 3, (new_pose[2]-.3,new_pose[2]+.3))
        self.particles = self.particles[1:]

        self.set_odometry_proxy(x=new_pose[0],y=new_pose[1],yaw=new_pose[2])

    def gen_particles(self, number_of_particles, center, radius, heading_range):
        #print "GENERATING PARTICLES:", number_of_particles

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
        self.publish_particle_array()

    def gen_guass_particles(self, number_of_particles, center, cov):
        self.particles = np.random.multivariate_normal(center,cov,number_of_particles)
        self.particles[:2] = np.clip(self.particles[:2],0,2.5)
        self.publish_particle_array()

    def run_filter(self):
        '''
        This function deals with actually running the filter.
        '''
        print "running"
        r = rospy.Rate(10) #hz
        start_time = time.time()
        while not rospy.is_shutdown():
            r.sleep()

            while len(self.laser_scan) == 0 and not rospy.is_shutdown():
                print "Waiting for scan."
                time.sleep(.5)
                continue

            self.particles += self.pose_update
            
            # Reset the pose update so that the next run will contain the pose update from this point
            self.pose_update = np.array([0,0,0], np.float32)
            
            #try:
            # weights holds particles weights. The more accurate a measurement was, the close to 1 it will be.
            weights_raw = self.m.generate_weights(self.particles,self.laser_scan)

            if np.mean(weights_raw) == 0:
                break

            # # # Remove low weights from particle and weights list
            weight_percentile = 95 #percent
            weights_indicies_to_keep = weights_raw > np.percentile(weights_raw,weight_percentile)
            weights = np.repeat(weights_raw,3).reshape(len(weights_raw),3)
            #weighted_particles = self.particles*weights/np.mean(weights_raw)
            self.particles = self.particles[weights_indicies_to_keep]

            # Pick the single best particle to spawn new ones from
            #best = self.particles[np.argmax(weights_raw)]

            #Just for debugging ==================================
            # print "WEIGHT PERCENTILE:", weight_percentile
            # print "CUTOFF:", np.percentile(weights_raw,weight_percentile)
            status = "PARTICLE COUNT: %i/%i"%(len(self.particles),self.MAX_PARTICLES)
            rospy.loginfo(status)
            #sigma = np.std(self.particles,axis=0) + np.array([.02,.02,.01])

            # Calculate pose esitmation before generating new particles
            new_pose = np.mean(self.particles, axis=0)

            # Update Pose
            self.publish_pose(new_pose)

            cov = np.eye(3)*.1**2
            translation_vairance = .1  #m  #.1
            rotational_vairance = .5 #rads #.5
            #self.gen_guass_particles(self.MAX_PARTICLES,new_pose,cov)
            self.gen_particles( self.MAX_PARTICLES - len(self.particles), 
                                new_pose[:2],
                                translation_vairance,
                                (new_pose[2]-rotational_vairance,new_pose[2]+rotational_vairance) )

            #except Exception,e:
            #     print "Error was found and excepted.",str(e)
            #     break
            # print 
        running = False

    def publish_pose(self,particle_avg):
        # Update pose and TF
        self.pose_est = np.array(particle_avg)

        #Generate pose
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose_est[2])
        header = Header(
            stamp=rospy.Time.now(),
            frame_id="map"
        )
        pose = Pose(
            position=Point(
                x=self.pose_est[0],
                y=self.pose_est[1],
                z=.127
            ),
            orientation=Quaternion(
                x=q[0],
                y=q[1],
                z=q[2],
                w=q[3],
            )
        )

        # Publish pose stamped
        self.pose_est_pub.publish(
            PoseStamped(
                header=header,
                pose=pose
            )
        )

        # self.br.sendTransform((self.pose_est[0], self.pose_est[1], .125), q,
        #          rospy.Time.now(),
        #          "base_link",
        #          "map")

        # Publish pose with covariance stamped.
        p_c_s = PoseWithCovarianceStamped()
        p_c = PoseWithCovariance()
        covariance = np.array([0.1,   0,   0,   0,   0,   0,
                                 0, 0.1,   0,   0,   0,   0,
                                 0,   0, 0.1,   0,   0,   0,
                                 0,   0,   0, 0.1,   0,   0,
                                 0,   0,   0,   0, 0.1,   0,
                                 0,   0,   0,   0,   0, 0.1])**2
        p_c.pose = pose
        p_c.covariance = covariance
        p_c_s.header = header
        p_c_s.header.frame_id = "map"
        p_c_s.pose = p_c
        self.p_c_s_est_pub.publish(p_c_s)


    def got_twist(self,msg):
        # Just a temp method to test the filter
        vehicle_twist = msg.twist.twist
        
        # time_since_last_msg = self.prev_time - time.time() #seconds
        # self.prev_time = time.time()
        # incoming_msg_freq = 10.0

        # This accounts for Shia's rotation - if he is pointed at a 45 degree angle and moves straight forward (which is what the twist message will say),
        # he is not moving directly along the x axis, he is moving at an offset angle.
        c, s = np.cos(self.pose_est[2]), np.sin(self.pose_est[2])
        rot_mat = np.matrix([
            [c,     -s],
            [s,      c],
        ], dtype=np.float32)
        # Then we add the x or y translation that we move, rounding down if it's super small
        x, y = np.dot(rot_mat, [vehicle_twist.linear.x, vehicle_twist.linear.y]).A1
        tolerance = 1e-7
        if abs(x) < tolerance: x = 0
        if abs(y) < tolerance: y = 0
        if abs(vehicle_twist.angular.z) < tolerance: vehicle_twist.angular.z = 0
        
        # By summing these components, we get an integral - converting velocity to position
        self.pose_update += np.array([x, y, vehicle_twist.angular.z])
        self.pose += np.array([x, y, vehicle_twist.angular.z])

    def got_odom(self,msg):
        # Update current pose based on odom data, note that this is only an estimation of Shia's position
        # The twist is a measure of velocity from the previous state

        vehicle_twist = msg.twist.twist

        # This accounts for Shia's rotation - if he is pointed at a 45 degree angle and moves straight forward (which is what the twist message will say),
        # he is not moving directly along the x axis, he is moving at an offset angle.
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        rot_mat = np.matrix([
            [c,     -s],
            [s,      c],
        ], dtype=np.float32)

        # Then we add the x or y translation that we move, rounding down if it's super small
        x, y = np.dot(rot_mat, [vehicle_twist.linear.x, vehicle_twist.linear.y]).A1

        tolerance = 1e-7
        if abs(x) < tolerance: x = 0
        if abs(y) < tolerance: y = 0
        if abs(vehicle_twist.angular.z) < tolerance: vehicle_twist.angular.z = 0
        
        # By summing these components, we get an integral - converting velocity to position
        self.pose_update += [x, y, vehicle_twist.angular.z]
        self.pose += [x, y, vehicle_twist.angular.z]

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

def start_navigation(msg):
    m = GPUAccMap(msg.map)
    init_pose = msg.init_pose
    f = GPUAccFilter(init_pose[:2], .5, (.5+init_pose[2],-.5+init_pose[2]), m)
    

# Set up start command subscriber and wait until we get that signal
rospy.init_node('particle_filter')
rospy.Subscriber("/robot/start_navigation", StartNavigation, start_navigation)
print "> Waiting for navigation start command..."

rospy.spin()