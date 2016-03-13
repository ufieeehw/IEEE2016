#!/usr/bin/env python
import rospy
from std_msgs.msg import Header, Bool, Float32
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import LaserScan
import tf

from ieee2016_msgs.srv import LidarSelector
from ieee2016_msgs.msg import StartNavigation

import struct
import binascii
import numpy as np
import math
import time
import random
import matplotlib.pyplot as plt
from threading import Thread
import os

import pyopencl as cl

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
        print len(packed)
        return packed

class GPUAccMap():
    def __init__(self,map_version):
        # Each line segment in the form: ax1, ay1, ax2, ay2, bx1, by1, bx2, by2, ...
        self.map = np.array(map_version).astype(np.float32)

        self.map_pub = rospy.Publisher("/lidar/scan_sim", LaserScan, queue_size=5)

        # LaserScan parameters
        self.angle_increment = .005 #rads/index
        self.min_angle = -3.14159274101 #rads
        self.max_angle = 3.14159274101
        self.max_range = 5.0 #m
        self.min_range = 0.1

        self.index_count = int((self.max_angle - self.min_angle)/self.angle_increment)
        # Set up pyopencl
        self.ctx = cl.create_some_context()
        self.queue = cl.CommandQueue(self.ctx)
        self.mf = cl.mem_flags
        # Load .cl program
        self.prg = cl.Program(self.ctx, """
        typedef struct lidar
        {   
            float4      coeff;
            float       min_distance;

        } lidar;

        float correct_distance(float dist, int offset, __global const lidar *l)
        {   
            float error;

            error = (l+offset)->coeff.s0 + 
                    (l+offset)->coeff.s1 * dist +
                    (l+offset)->coeff.s2 * pow(dist,2)+
                    (l+offset)->coeff.s3 * pow(dist,3);

            return dist-error;
        }

        __kernel void trace( __global const float *p,  
                             __global const float *walls,
                             __global const lidar *l,
                             __global const int  *angle_to_lidar,
                                      const uint  wall_cnt,
                             __global const float *angles_to_check,
                                      const uint  angle_cnt,
                             __global       float *temp_out,
                             __global       float *weights) 
        {

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
                    float3 point1 = (float3)(walls[5*w],walls[5*w+1],0);
                    float3 point2 = (float3)(walls[5*w+2],walls[5*w+3],0);

                    v1 = ray_origin - point1;
                    v2 = point2 - point1;
                    float3 v3 = (float3)(-ray_direction.s1, ray_direction.s0, 0);

                    v2_dot_v3 = dot(v2,v3);
                    if(v2_dot_v3 != 0){
                        t1 = cross(v2, v1) / v2_dot_v3;
                        t2 = dot(v1, v3) / v2_dot_v3;
                        d = t1.s2;
                        if(d >= 0.0 && t2 >= 0.0 && t2 <= 1.0 && d > (l+angle_to_lidar[a])->min_distance){
                            if(walls[5*w+4] == 1){
                                d = correct_distance(d, angle_to_lidar[a], l);
                            }
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
        fov = math.radians(40)
        self.deg_index = int(fov/self.angle_increment)
        self.indicies_to_compare = np.array([], np.int32)
        self.angle_to_lidar = np.array([], np.uint32)
        self.step = 1

        # Define the lidar Corrector, and init the lidars - order is important.
        # The addition to the min_range is to account for the fact the lidars are offset from base_link
        lidar = LidarCorrector()
        lidar.add_lidar([.052133885,.0298194301,-.0783299411,.0249704984],self.min_range+.1)       # Back
        lidar.add_lidar([-.0102461432,.0205318341,.0208797008,-.00183668788],self.min_range+.125)    # Left
        lidar.add_lidar([.0451106369,-.1020802842,.1752494951,-.0700421761],self.min_range+.125)     # Front
        lidar.add_lidar([.0118628147,.1439880093,-.1344849981,.0298791165],self.min_range+.1)      # Right
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

        #self.indicies_to_compare = np.arange(1256,step=8)

        # To generate weights, we need those indices to be pre-converted to radian angle measures 
        self.angles_to_compare = (self.indicies_to_compare*self.angle_increment + self.min_angle).astype(np.float32)

        self.map_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.map)
        self.lidar_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=packed_lidar)
        self.angle_to_lidar_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angle_to_lidar.astype(np.int32))
        self.angles_to_compare_cl = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=self.angles_to_compare)

    def change_indicies_to_compare(self,srv):
        lidars = srv.lidar_names

        # This look like a big mess, and it is.
        # We add the indicies of each LIDAR's FOV to the master list depending if it was selected or not
        self.indicies_to_compare = np.array([])
        if "right" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count/4 - self.deg_index, self.index_count/4 + self.deg_index, step=self.step) )
        if "front" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count/2 - self.deg_index, self.index_count/2 + self.deg_index, step=self.step) )
        if "left" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(3*self.index_count/4 - self.deg_index, 3*self.index_count/4 + self.deg_index, step=self.step) )
        if "back" in lidars:
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(0, self.deg_index, step=self.step))
            self.indicies_to_compare = np.append( self.indicies_to_compare,            
                np.arange(self.index_count - self.deg_index, self.index_count, step=self.step))

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
        temp_cl = cl.Buffer(self.ctx, self.mf.WRITE_ONLY, weights.nbytes)

        self.prg.trace(self.queue, (1,), None, point_cl,
                                            self.map_cl, 
                                          self.lidar_cl,
                                 self.angle_to_lidar_cl,
                             np.uint32(self.map.size/5), 
                              self.angles_to_compare_cl, 
                 np.uint32(self.angles_to_compare.size),
                                                temp_cl, 
                                             weights_cl).wait()

        cl.enqueue_copy(self.queue, weights, weights_cl).wait()
        cl.enqueue_copy(self.queue, temp, temp_cl).wait()
        #print np.degrees(np.where(temp>0)[0]*self.angle_increment)
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
    def __init__(self, starting_point, m, m_a):
        # ROS inits
        #rospy.Subscriber('/robot/pf_pose_est', PoseStamped, self.got_pose)        
        self.tf_broad = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.m = m
        self.m_a = m_a
        
        self.pose = np.array(starting_point).astype(np.float32)

        self.tf_listener.waitForTransform("map","base_link", rospy.Time(0), rospy.Duration(1.0))

        rate = rospy.Rate(25) #hz
        while not rospy.is_shutdown():

            time = self.tf_listener.getLatestCommonTime("map","base_link")
            pos, quaternion = self.tf_listener.lookupTransform("map","base_link", time)
            rot = tf.transformations.euler_from_quaternion(quaternion)

            self.pose = np.array([pos[0],pos[1],rot[2]])

            self.control()
            rate.sleep()

    
    def control(self):
        self.m.simulate_scan(self.pose.astype(np.float32))
        #self.m_a.simulate_scan(self.pose.astype(np.float32))

    def got_pose(self, msg):
        yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])[2]
        self.pose = np.array([msg.pose.position.x,msg.pose.position.y,yaw])


def start_navigation(msg):
    m = GPUAccMap(msg.map)
    #m_a = GPUAccMapAct(msg.map)
    print "> Starting navigation."
    s = Simulator(msg.init_pose,m,m)
    
rospy.init_node('lidar_sim', anonymous=True)
rospy.Subscriber("/robot/start_navigation", StartNavigation, start_navigation)
print "Waiting for start_navigation..."

rospy.spin()
