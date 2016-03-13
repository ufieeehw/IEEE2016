#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool, Int8, Header
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3, Point32, PointStamped
from ieee2016_msgs.srv import NavWaypoint, ArmWaypoint
import tf

roslib.load_manifest('ieee2016_vision')
from camera_manager import Camera
from block_manager import EndEffector, BlockServer, WaypointGenerator
from waypoint_utils import load_waypoints, update_waypoints
from qr_detector import DetectQRCodeTemplateMethod

# Temp Imports
roslib.load_manifest('ieee2016_simulator_new')
import random

from kd_tree import KDTree
import numpy as np
import time
import os

class temp_GenerateBlockPoints():
    '''
    Temp way of generating blocks.
    Generate point cloud and a block_tree
    '''
    def __init__(self,inital_blocks):
        self.point_cloud_pub = rospy.Publisher("/camera/block_point_cloud", PointCloud, queue_size=2) 
        self.point_cloud_pub_diag = rospy.Publisher("/camera/diag_block_point_cloud", PointCloud, queue_size=2) 
        self.initial_blocks = inital_blocks

        self.dx = .0635
        self.dy = .0629   # Used for half blocks
        self.dz = .0381
        
        self.y = 2.153

    def generate_c_blocks(self):
        self.c_tree = KDTree(.0381/2)
        base_x = .19
        base_z = .20
        colors = ["blue","blue","blue","blue",
                  "red","red","red","red",
                  "green","green","green","green",
                  "yellow","yellow","yellow","yellow"]
        
        random.shuffle(colors)

        for i in range(self.initial_blocks/2):
            this_x = base_x + self.dx*i
            self.c_tree.insert_unique([this_x,self.y,base_z],colors[i])
            self.c_tree.insert_unique([this_x,self.y,base_z+self.dz],colors[i+self.initial_blocks/2])
        
        self.publish_points(self.c_tree,self.point_cloud_pub)
        return self.c_tree

    def generate_b_blocks(self):
        '''
        Block order (from front view)

        (first front layer)
            00 01 02 03 04 05 06 07
            08 09 10 11 12 13 14 15

        (second back layer)
            16 17 18 19 20 21 22 23
            24 25 26 27 28 29 30 31
        '''
        self.b_tree_diag = KDTree(.0381/2)
        base_x = .90
        base_z = .25

        blocks = [["red","red","red","red",
                   "blue","blue","blue","blue",
                   "green","green","green","green",
                   "yellow","yellow","yellow","yellow"],
                  ["red","red","blue","blue",
                   "green","green","yellow","yellow"]]
        
        random.shuffle(blocks[1])
        random.shuffle(blocks[0])
        
        # At these indicies are where we are going to put the half blocks.
        half_locations = np.array(random.sample(range(0, 16), 4))

        # Populate blocks list with full and half blocks.
        self.b_blocks = np.full(32,"none",dtype=object)
        for i in range(16):
            if i in half_locations:
                self.b_blocks[i] = blocks[1][0]
                del blocks[1][0]
                self.b_blocks[i+16] = blocks[1][0]
                del blocks[1][0]
            else:
                self.b_blocks[i] = blocks[0][0]
                del blocks[0][0]

        # Go through each dimension and add it to the tree. This is the diagnostics tree, not what's visible to the camera.
        for i in range(8):
            this_x = base_x + self.dx*i
            self.b_tree_diag.insert_unique([this_x,self.y,base_z+self.dz],self.b_blocks[i])
            self.b_blocks[i] = [[this_x,self.y,base_z+self.dz],self.b_blocks[i]]
            #print ('%7s')%self.b_blocks[i],
        #print
        for i in range(8,16):
            this_x = base_x + self.dx*(i-8)
            self.b_tree_diag.insert_unique([this_x,self.y,base_z],self.b_blocks[i])
            self.b_blocks[i] = [[this_x,self.y,base_z],self.b_blocks[i]]
            #print ('%7s')%self.b_blocks[i],
        #print
        #print
        for i in range(16,24):
            this_x = base_x + self.dx*(i-16)
            self.b_tree_diag.insert_unique([this_x,self.y+self.dy,base_z+self.dz],self.b_blocks[i])
            self.b_blocks[i] = [[this_x,self.y+self.dy,base_z+self.dz],self.b_blocks[i]]
            #print ('%7s')%self.b_blocks[i],
        #print
        for i in range(24,32):
            this_x = base_x + self.dx*(i-24)
            self.b_tree_diag.insert_unique([this_x,self.y+self.dy,base_z],self.b_blocks[i])
            self.b_blocks[i] = [[this_x,self.y+self.dy,base_z],self.b_blocks[i]]
            #print ('%7s')%self.b_blocks[i],
        #print

        self.publish_points(self.b_tree_diag,self.point_cloud_pub_diag)

    def generate_b_camera_view(self):
        # This will take the current list of points and generate the frontal view, i.e. what the camera can see.
        self.b_tree = KDTree(.0381/2)
        base_x = .90
        base_z = .25


        # Populates tree with the frontmost blocks
        for i in range(8):
            this_x = base_x + self.dx*i
            if self.b_blocks[i][1] == "none":
                self.b_tree.insert_unique([this_x, self.y+self.dy, base_z+self.dz],self.b_blocks[i+16][1])
            else:
                self.b_tree.insert_unique([this_x, self.y, base_z+self.dz],self.b_blocks[i][1])

        for i in range(8,16):
            this_x = base_x + self.dx*(i-8)
            if self.b_blocks[i][1] == "none":
                self.b_tree.insert_unique([this_x, self.y+self.dy, base_z],self.b_blocks[i+16][1])
            else:
                self.b_tree.insert_unique([this_x, self.y, base_z],self.b_blocks[i][1])

        #print self.b_tree.nodes
        self.publish_points(self.b_tree,self.point_cloud_pub)
        return self.b_tree

    def remove_b_blocks(self,indicies):
        for i in indicies:
            self.b_blocks[i] = "none"

        return self.generate_b_camera_view()

    def generate_a_blocks(self):
        self.a_tree = KDTree(.0381/2)
        base_x = 1.73
        base_z = .14

        for i in range(self.initial_blocks/2):
            this_x = base_x + self.dx*i
            self.a_tree.insert_unique([this_x,self.y,base_z],"blue")
            self.a_tree.insert_unique([this_x,self.y,base_z+self.dz],"blue")
        
        self.publish_points(self.a_tree,self.point_cloud_pub)
        return self.a_tree

    def publish_points(self, tree, topic):
        points = []
        channels = [[],[],[]]
        for p in tree.nodes:
            #print p.linked_object
            if p.linked_object == "blue":
                channels[0].append(0) #R
                channels[1].append(0) #G
                channels[2].append(1) #B
            elif p.linked_object  == "red":
                channels[0].append(1) #R
                channels[1].append(0) #G
                channels[2].append(0) #B
            elif p.linked_object  == "green":
                channels[0].append(0) #R
                channels[1].append(1) #G
                channels[2].append(0) #B
            elif p.linked_object  == "yellow":
                channels[0].append(1) #R
                channels[1].append(1) #G
                channels[2].append(0) #B
            elif p.linked_object  == "none":
                channels[0].append(0) #R
                channels[1].append(0) #G
                channels[2].append(0) #B

            points.append(Point32(*p.point))

        rgb_channels = [ChannelFloat32(name="r", values=channels[0]),ChannelFloat32(name="g", values=channels[1]),ChannelFloat32(name="b", values=channels[2])]
        time.sleep(1.5)
        topic.publish(PointCloud(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                    ),
                points=points,
                channels=rgb_channels
            )
        )

class Tester():
    def __init__(self):
        rospy.init_node("unit_tester")

        # Misc Objects
        self.point_cloud_generator = temp_GenerateBlockPoints(16)

        print "> Initialization Complete."

    def run_test(self):
        rate = rospy.Rate(1) #hz
        while not rospy.is_shutdown():
            rospy.loginfo("Publishing...")
            self.point_cloud_generator.generate_b_blocks()
            print self.point_cloud_generator.b_tree_diag
            rate.sleep()

if __name__ == "__main__":
    t = Tester()
    t.run_test()

    rospy.spin()