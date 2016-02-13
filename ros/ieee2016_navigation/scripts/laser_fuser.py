#!/usr/bin/env python
import numpy as np
import math

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
import message_filters

class LaserFuser():
    def __init__(self):
        self.scan_pub = rospy.Publisher('scan_fused', LaserScan, queue_size=3)
        self.scan_pc_pub = rospy.Publisher('scan_fused_pc', PointCloud, queue_size=3)
        rospy.init_node('laser_fuser')

        # Start a TF listener and make sure it gets data before preceding
        self.tf_listener = tf.TransformListener()
        print
        print " > Waiting for TF..."
        self.tf_listener.waitForTransform("base_link","laser_left", rospy.Time(0), rospy.Duration(5.0))
        self.tf_listener.waitForTransform("base_link","laser_middle", rospy.Time(0), rospy.Duration(5.0))
        self.tf_listener.waitForTransform("base_link","laser_right", rospy.Time(0), rospy.Duration(5.0))
        self.frames = ["laser_left","laser_middle","laser_right"]
        self.FOVs = [100,100,100] #degrees
        print " > TF Found! Starting Fuser."

        # Define our message filters - these will make sure the laserscans come in at the same time
        left_sub = message_filters.Subscriber('scan_left',LaserScan)
        middle_sub = message_filters.Subscriber('scan_middle',LaserScan) 
        right_sub = message_filters.Subscriber('scan_right',LaserScan)
        mf_ts = message_filters.ApproximateTimeSynchronizer([left_sub,middle_sub,right_sub], 10, .1)
        mf_ts.registerCallback(self.got_scans)

        # Define new laserscan
        self.fused_scan = LaserScan()
        self.fused_scan.header.frame_id = "laser_fused"
        self.fused_scan.angle_increment = .007 #rads/index
        self.fused_scan.angle_min = -3.14159274101 #rads
        self.fused_scan.angle_max = 3.14159274101 #rads
        self.fused_scan.range_max = 5.0 #m
        self.fused_scan.range_min = 0.0002 #m

        rospy.spin()

    def got_scans(self,*scans):
        # Take all three scans, trim to a new FOV, convert them to cartesian, apply transformations, pub pointcloud, tranform back to cart, and pub laserscan

        fused_cartesian = np.array([-1,-1])
        for i,s in enumerate(scans):
            # Define parameters based on scan data
            angle_increment = s.angle_increment
            ranges = np.array(s.ranges)
            ranges_size = ranges.size

            # Trim to FOV
            new_FOV = math.radians(self.FOVs[i])
            angle_min = -(new_FOV/2.0)
            angle_max = (new_FOV/2.0)
            ranges = ranges[int(ranges_size/2 + angle_min/angle_increment): int(ranges_size/2 + angle_max/angle_increment)]
            ranges_size = ranges.size

            #self.pubish_scan(s,ranges,self.frames[i],angle_min,angle_max,angle_increment)

            # Get TF data about the LIDAR position
            (trans,q_rot) = self.tf_listener.lookupTransform('base_link',self.frames[i], rospy.Time(0))
            rot = tf.transformations.euler_from_quaternion(q_rot)

            # Generate list that maps indices of the ranges to radian angle measures
            thetas = np.linspace(angle_min, angle_max, ranges.size, endpoint=True) + rot[2]
            cartesian_scan = np.apply_along_axis(self.polar_to_cart,0,np.vstack((ranges,thetas)),trans,self.frames[i]).T# + np.array(trans[:2])
            
            # Add this scan's points to the list of x,y points
            fused_cartesian = np.vstack((fused_cartesian,cartesian_scan))

        #print ranges_size*3
        self.fused_cartesian = fused_cartesian[1:]
        self.publish_pc(self.fused_cartesian)

        # Now we convert back to polar and publish
        # Steal a timestamp from one of the real laserscans so we are in the right timeframe
        self.fused_scan.header.stamp = scans[1].header.stamp
        self.fused_scan.ranges = self.cart_to_polar()
        self.scan_pub.publish(self.fused_scan)

    def polar_to_cart(self,r_theta,translation,frame):
        # Convert (r,theta) into (x,y)
        # Since the LIDAR are flipped and rotated in different directions, you have to change the signs of x and y based on the scan
        if frame == "laser_middle":
            x = (r_theta[0] * np.cos(r_theta[1]) + translation[0])
            y = -(r_theta[0] * np.sin(r_theta[1]) + translation[1])
        else:
            x = -r_theta[0] * np.cos(r_theta[1]) + translation[0]
            y = r_theta[0] * np.sin(r_theta[1]) + translation[1]

        return x,y


    def cart_to_polar(self):
        x_y = self.fused_cartesian
        # Generate the angles associated with each indices.
        indices = int((self.fused_scan.angle_max-self.fused_scan.angle_min)/self.fused_scan.angle_increment)+1

        polar_points_angles = np.linspace(self.fused_scan.angle_min,self.fused_scan.angle_max,indices)
        polar_points_bins = np.zeros(indices) + np.inf
        
        # Convert (x,y) to (r,theta). It's faster to use all numpy functions.
        r_theta = np.vstack((np.sqrt(x_y[:,0]**2 + x_y[:,1]**2),np.arctan2(x_y[:,1], x_y[:,0]))).T
        
        # Find which indices each (r,theta) point belongs in and put it there.
        # Note: if there is more then one r associated with the same bin, the value will get overwritten NOT averaged
        sorted_indices = np.digitize(r_theta[:,1],polar_points_angles)
        polar_points_bins[sorted_indices-1] = r_theta[:,0]

        return polar_points_bins

    def publish_pc(self,points,frame_id="laser_fused"):
        pc_points = []
        for p in points:
            if np.isfinite(p.any()): pc_points.append(Point32(x=p[0],y=p[1],z=0))

        self.scan_pc_pub.publish(PointCloud(
            header = Header(
                stamp=rospy.Time.now(),
                frame_id=frame_id
                ),
            points=pc_points
            )
        )

l = LaserFuser()
