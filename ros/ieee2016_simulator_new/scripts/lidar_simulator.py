#!/usr/bin/env python
import rospy
from std_msgs.msg import Header, Bool
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import LaserScan
import tf

from ieee2016_msgs.srv import LidarSelector

import numpy as np
import math
import time
import random
import matplotlib.pyplot as plt
from threading import Thread
import os

import pyopencl as cl

class GPUAccMap():
    def __init__(self):
        # Each line segment in the form: ax1, ay1, ax2, ay2, bx1, by1, bx2, by2, ...
        self.map = np.array([0, 0, 0, .784, 0, .784, .015, .784, .015, .784, .015, 1.158, 0, 1.158, .015, 1.158, 0, 1.158, 0, 2.153, .464, .784, .479, .784, .479, .784, .479, 1.158, .464, .784, .464, 1.158, .464, 1.158, .479, 1.158, 0, 0, .549, 0, .549, 0, .549, .317, .549, .317, .569, .317, .569, 0, .569, .317, .569, 0, .809, 0, .809, 0, .809, .317, .809, .317, .829, .317, .829, 0, .829, .317, .829, 0, 2.458, 0, 0, 2.153, 2.458, 2.153, 2.458, 0, 2.458, .907, 2.161, .907, 2.458, .907, 2.161, .907, 2.161, 1.178, 2.161, 1.178, 2.458, 1.178, 2.458, 1.178, 2.458, 1.181, 2.161, 1.181, 2.458, 1.181, 2.161, 1.181, 2.161, 1.452, 2.161, 1.452, 2.458, 1.452, 2.458, 1.452, 2.458, 1.482, 2.161, 1.482, 2.458, 1.482, 2.161, 1.482, 2.161, 1.753, 2.161, 1.753, 2.458, 1.753, 2.458, 1.753, 2.458, 1.783, 2.161, 1.783, 2.458, 1.783, 2.161, 1.783, 2.161, 2.054, 2.161, 2.054, 2.458, 2.054, 2.458, 2.054, 2.458, 2.153]).astype(np.float32)

        self.map_pub = rospy.Publisher("/lidar/scan_sim", LaserScan, queue_size=5)

        # LaserScan parameters
        self.angle_increment = .005 #rads/index
        self.min_angle = -3.14159274101 #rads
        self.max_angle = 3.14159274101
        self.max_range = 5.0 #m
        self.min_range = 0.00999999977648

        self.index_count = int((self.max_angle - self.min_angle)/self.angle_increment)
        # Set up pyopencl
        self.ctx = cl.create_some_context()
        self.queue = cl.CommandQueue(self.ctx)
        self.mf = cl.mem_flags
        # Load .cl program
        self.prg = cl.Program(self.ctx, """
        __kernel void trace( __global const float *p,  
                             __global const float *walls,
                                      const uint  wall_cnt,
                             __global const float *angles_to_check,
                                      const uint  angle_cnt,
                             __global const float *misfit_actual,
                             __global       float *weights
            ) {

            int i = get_global_id(0);

            float min;
            float3 ray_origin = (float3)(p[i*3], p[i*3+1],0);

            float3 t1,v1,v2;
            float v2_dot_v3;
            float t2,d,theta;
            float temp;
            temp = p[0];

            for(int a = 0; a < angle_cnt; a++){
                min = 1000;
                theta = p[i*3+2] + angles_to_check[a];
                float3 ray_direction = (float3)(cos(theta), sin(theta),0);

                for(int w = 0; w < wall_cnt; w++){
                    float3 point1 = (float3)(walls[4*w],walls[4*w+1],0);
                    float3 point2 = (float3)(walls[4*w+2],walls[4*w+3],0);

                    v1 = ray_origin - point1;
                    v2 = point2 - point1;
                    float3 v3 = (float3)(-ray_direction.s1, ray_direction.s0, 0);

                    v2_dot_v3 = dot(v2,v3);
                    if(v2_dot_v3 != 0){
                        t1 = cross(v2, v1) / v2_dot_v3;
                        t2 = dot(v1, v3) / v2_dot_v3;
                        d = t1.s2;
                        if(d >= 0.0 && t2 >= 0.0 && t2 <= 1.0){
                            if(d < min){
                                min = d;
                            }
                        }
                    }
                }
                weights[a] = min;
            }
            
        }
            """).build()

        # Only ranges at these indicies will be checked
        # Pick ranges around where the LIDAR scans actually are (-90,0,90) degrees
        self.deg_index = int(math.radians(30)/self.angle_increment)
        self.indicies_to_compare = np.array([], np.int32)
        self.step = 1

        # The serivce to determine which LIDAR to navigate with
        #rospy.Service('/robot/navigation/select_lidar', LidarSelector, self.change_indicies_to_compare)
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(self.index_count/4 - self.deg_index, self.index_count/4 + self.deg_index, step=self.step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(self.index_count/2 - self.deg_index, self.index_count/2 + self.deg_index, step=self.step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(3*self.index_count/4 - self.deg_index, 3*self.index_count/4 + self.deg_index, step=self.step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(0, self.deg_index, step=self.step))
        self.indicies_to_compare = np.append( self.indicies_to_compare,            
            np.arange(self.index_count - self.deg_index, self.index_count, step=self.step))

        #self.indicies_to_compare = np.arange(1256,step=8)

        # To generate weights, we need those indices to be pre-converted to radian angle measures 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)

        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)
        self.map_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.map)

    def change_indicies_to_compare(self,srv):
        lidars = srv.lidar_names

        # This look like a big mess, and it is.
        # We add the indicies of each LIDAR's FOV to the master list depending if it was selected or not
        self.indicies_to_compare = np.array([])
        if "right" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count/4 - self.deg_index, self.index_count/4 + self.deg_index, step=self.step) ).astype(np.int)
        if "front" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count/2 - self.deg_index, self.index_count/2 + self.deg_index, step=self.step) ).astype(np.int)
        if "left" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(3*self.index_count/4 - self.deg_index, 3*self.index_count/4 + self.deg_index, step=self.step) ).astype(np.int)

        # Update other necessary parameters 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)
        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)
        print "Change",self.indicies_to_compare
        return True

    def simulate_scan(self, point):
        # Not really weights, just a holder for the ranges
        weights = np.zeros(self.indicies_to_compare.size).astype(np.float32)
        weights_cl = cl.Buffer(self.ctx, self.mf.WRITE_ONLY, weights.nbytes)

        point_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=point)
        temp = np.zeros(self.indicies_to_compare.size).astype(np.float32)
        temp_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=temp)

        self.prg.trace(self.queue, (1,), None, point_cl, 
                                            self.map_cl, 
                             np.uint32(self.map.size/4), 
                              self.angles_to_compare_cl, 
                 np.uint32(self.angles_to_compare.size),
                                                temp_cl, 
                                             weights_cl).wait()

        cl.enqueue_copy(self.queue, weights, weights_cl).wait()
        #print weights

        # Just for publishing laserscans
        ranges = np.zeros(self.index_count)
        #print self.indicies_to_compare
        ranges[self.indicies_to_compare] = weights
        #ranges = add_noise(ranges,.05)
        self.map_pub.publish(LaserScan(    
            header=Header(
                stamp = rospy.Time.now(),
                frame_id = "laser_fused",
                ),
            angle_min=self.min_angle,
            angle_max=self.max_angle,
            angle_increment=self.angle_increment,
            time_increment=0,
            scan_time=0,
            range_min=self.min_range,
            range_max=self.max_range,
            ranges=ranges.tolist(),
            intensities=[],
            )
        )

class Simulator():
    def __init__(self, starting_point, m):
        # ROS inits
        rospy.Subscriber('/robot/pf_pose_est', PoseStamped, self.got_pose)        
        self.tf_broad = tf.TransformBroadcaster()
        
        self.m = m
        
        self.pose = np.array(starting_point).astype(np.float32)

        rate = rospy.Rate(25) #hz
        while not rospy.is_shutdown():
            self.last_time = time.time()
            self.control()

            rate.sleep()

    
    def control(self):
        self.m.simulate_scan(self.pose.astype(np.float32))

    def got_pose(self, msg):
        yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])[2]
        self.pose = np.array([msg.pose.position.x,msg.pose.position.y,yaw])

def add_noise(values,noise_size):
    # Adds noise to scan to some data
    return values + np.random.uniform(0,noise_size-noise_size/2.0,values.size)

rospy.init_node('test_particle_filter', anonymous=True)
s = Simulator([.2,.2,1.57],GPUAccMap())