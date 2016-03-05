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
from block_manager import EndEffector, BlockServer, ProcessBlocks
from waypoint_utils import load_waypoints, update_waypoints
from qr_detector import DetectQRCodeTemplateMethod

# Temp Imports
roslib.load_manifest('ieee2016_simulator_new')
import random

from kd_tree import KDTree
import numpy as np
import time
import os

class temp_ProcessTrainBoxes():
    '''
    Just a placeholder for the train box assigner
    '''
    def __init__(self,waypoints):
        self.point_cloud_pub = rospy.Publisher("/camera/train_boxes", PointCloud, queue_size=2) 
        self.waypoints = waypoints
        print "> Train Proessor Ready."

    def process_train_blocks(self):
        colors = ["red","blue","green","yellow"]
        random.shuffle(colors)
        raw_points = [self.waypoints['box_1'],self.waypoints['box_2'],self.waypoints['box_3'],self.waypoints['box_4']]
        #print colors
        self.train_points = []
        for point,color in zip(raw_points,colors):
            self.train_points.append([color,point])

        print "> Saving Train Waypoints"
        self.publish_points()
        update_waypoints(['box_1','box_2','box_3','box_4'],colors)

    def publish_points(self):
        points = []
        channels = [[],[],[]]
        for p in self.train_points:
            if p[0] == "blue":
                #rgbs.append("FF0000")#struct.pack('i', 0x0000ff))
                channels[0].append(0) #R
                channels[1].append(0) #G
                channels[2].append(1) #B
            elif p[0] == "red":
                #rgbs.append(struct.unpack('f', struct.pack('i', 0xff0000))[0])
                channels[0].append(1) #R
                channels[1].append(0) #G
                channels[2].append(0) #B
            elif p[0] == "green":
                #rgbs.append(struct.unpack('f', struct.pack('i', 0x00ff00))[0])
                channels[0].append(0) #R
                channels[1].append(1) #G
                channels[2].append(0) #B
            elif p[0] == "yellow":
                #rgbs.append(struct.unpack('f', struct.pack('i', 0xffff00))[0])
                channels[0].append(1) #R
                channels[1].append(1) #G
                channels[2].append(0) #B

            points.append(Point32(
                    x=p[1][0],
                    y=p[1][1],
                    z=0
                )
            )
        rgb_channels = [ChannelFloat32(name="r", values=channels[0]),ChannelFloat32(name="g", values=channels[1]),ChannelFloat32(name="b", values=channels[2])]
        self.point_cloud_pub.publish(PointCloud(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                    ),
                points=points,
                channels=rgb_channels
            )
        )

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
            this_x = base_x + self.dz*i
            self.a_tree.insert_unique([this_x,self.y,base_z],colors[i])
            self.a_tree.insert_unique([this_x,self.y,base_z+self.dz],colors[i+self.initial_blocks/2])
        
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
            #print ('%7s')%self.b_blocks[i],
        #print
        for i in range(8,16):
            this_x = base_x + self.dx*(i-8)
            self.b_tree_diag.insert_unique([this_x,self.y,base_z],self.b_blocks[i])
            #print ('%7s')%self.b_blocks[i],
        #print
        #print
        for i in range(16,24):
            this_x = base_x + self.dx*(i-16)
            self.b_tree_diag.insert_unique([this_x,self.y+self.dy,base_z+self.dz],self.b_blocks[i])
            #print ('%7s')%self.b_blocks[i],
        #print
        for i in range(24,32):
            this_x = base_x + self.dx*(i-24)
            self.b_tree_diag.insert_unique([this_x,self.y+self.dy,base_z],self.b_blocks[i])
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
            if self.b_blocks[i] == "none":
                self.b_tree.insert_unique([this_x, self.y+self.dy, base_z+self.dz],self.b_blocks[i+16])
            else:
                self.b_tree.insert_unique([this_x, self.y, base_z+self.dz],self.b_blocks[i])

        for i in range(8,16):
            this_x = base_x + self.dx*(i-8)
            if self.b_blocks[i] == "none":
                self.b_tree.insert_unique([this_x, self.y+self.dy, base_z],self.b_blocks[i+16])
            else:
                self.b_tree.insert_unique([this_x, self.y, base_z],self.b_blocks[i])

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
        time.sleep(.5)
        topic.publish(PointCloud(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                    ),
                points=points,
                channels=rgb_channels
            )
        )

class ShiaStateMachine():
    '''
    This is the main program that will run when the start button is pressed.
    This is still a work in progress.
    '''
    def __init__(self):
        self.ros_manager = ros_manager(self)

        self.current_state = 0
        self.map_version = 0

        self.running = False
        self.waypoints = load_waypoints()

        # Defining Shia's limbs. 
        # 1 is on the right, 2 is on the left
        self.ee1 = EndEffector(gripper_count=4, ee_number=1, cam_position=2)
        self.ee2 = EndEffector(gripper_count=4, ee_number=2, cam_position=2)

        self.cam1 = Camera(cam_number=1)
        self.cam2 = Camera(cam_number=2)

        # Objects for detecting and returning location of QR codes. Parameters are the set distances from the codes (cm).
        self.qr_detector = DetectQRCodeTemplateMethod([57])
        self.block_processor = ProcessBlocks()

        # Misc Objects
        self.train_box_processor = temp_ProcessTrainBoxes(self.waypoints)
        self.point_cloud_generator = temp_GenerateBlockPoints(16)
        self.point_cloud_generator.generate_b_blocks()

        print "> State Machine Init Complete."
        print "> Waiting for map selection and start command."

        rospy.spin()

    def begin_2(self):
        '''
        This will run the map configuration where we start on the left side of the map.
        '''
        print
        print "> Running"
        print "> Map Version:",self.map_version
        print

        self.current_state+=1

        self.running = True
        rate = rospy.Rate(25) #hz
        while not rospy.is_shutdown():
            
            self.ros_manager.publish_current_state(self.current_state)

            if self.current_state == 1:
                '''
                The goals of this state are:
                    1. Drive through the box.
                    2. Begin processing the location of train blocks.
                    3. Move to location to process B blocks.
                '''
                print "=== STATE 1 ==="
                self.ros_manager.set_waypoint(self.waypoints['through_box'])
                self.train_box_processor.process_train_blocks()
                self.ros_manager.set_waypoint(self.waypoints['process_b_1'])

                self.current_state += 1

            elif self.current_state == 2:
                '''
                The goals of this state are:
                    1. Begin processing B blocks.
                    2. Move to the next B blocks processing location and continue processing.
                    3. Continue going to B block waypoints until all blocks are processed.
                '''
                print "=== STATE 2 ==="

                b_tree = self.point_cloud_generator.generate_b_camera_view()
                #ProcessBlocks(b_tree)
                #self.ros_manager.set_waypoint(self.waypoints['process_b_2'])

                self.current_state += 1

            elif self.current_state == 3:
                '''
                The goals of this state are:
                    1. Generate arm waypoint based on block locations.
                    2. Move to pick up first set of blocks.
                    3. Move back and pick up second set of blocks.
                '''
                print "=== STATE 3 ==="
                self.point_cloud_generator.remove_b_blocks([0,1,2,3])

                break

            elif self.current_state == 4:
                '''
                The goals of this state are:
                    1. Move to train box_1 (unless we aren't hold that color in the first end effector.
                    2. Release that color on the first end effector.
                    3. Move to next box with a color that we have. Repeat until at the last box.
                    5. Turn around and repeat with the other end effector - starting from box 4.
                '''
                print "=== STATE 4 ==="

            elif self.current_state == 5:
                '''
                The goals of this state are:
                    1. Move back to processing B blocks.
                    2. Find out if there are any half blocks we have to deal with.
                    3. Generate next arm waypoint (with half blocks considered).
                    4. Repeat for other end effector.

                After this state return to state 4 and repeat until there are no more blocks.
                '''
                print "=== STATE 5 ==="


            # elif self.current_state == 5:
            #     self.dectect_box_colors()
            # elif self.current_state == 6:
            #     self.unload_blocks_by_color()
            # elif self.current_state == 7:
            #     self.return_to_block()

            if self.current_state > 5: self.current_state = 0
            rate.sleep()

class ros_manager():
    def __init__(self,s):
        self.state_machine = s
        self.map_version_set = False

        # ROS inits
        self.nav_waypoint_pub = rospy.Publisher("/robot/waypoint", PoseStamped, queue_size=1)
        self.state_pub = rospy.Publisher("/robot/current_state", Int8, queue_size=1)
        self.nav_start_pub = rospy.Publisher("/robot/start_navigation", Int8, queue_size=1)
        self.nav_waypoint = rospy.ServiceProxy('/robot/nav_waypoint', NavWaypoint)
    
        rospy.init_node('main_control')
        
        rospy.Subscriber('/settings/start_command',Bool,self.recieve_start_command)
        rospy.Subscriber('/settings/map_version',Int8,self.determine_map_version)

    def recieve_start_command(self,msg):
        if msg.data and not self.state_machine.running and self.map_version_set:
            print "> State Machine Starting..."

            # 1 is the map configuration where we start on the right, 2 is on the left.
            if self.state_machine.map_version == 1:
                self.nav_start_pub.publish(Int8(data=1))
                self.state_machine.begin_1()
            elif self.state_machine.map_version == 2:
                self.nav_start_pub.publish(Int8(data=2))
                self.state_machine.begin_2()

    def determine_map_version(self,msg):
        self.state_machine.map_version = msg.data
        self.map_version_set = True

        print "> Map Version:",self.state_machine.map_version

    def set_waypoint(self,waypoint):
        q = tf.transformations.quaternion_from_euler(0, 0, waypoint[2])
        p_s = PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                ),
                pose=Pose(
                    position=Point(
                        x=waypoint[0],
                        y=waypoint[1],
                        z=.129
                    ),
                    orientation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3],
                    )
                )
            )
        self.nav_waypoint_pub.publish(p_s)
        print "> Waypoint published:",waypoint
        print "> Moving..."
        success = self.nav_waypoint(p_s)
        print "> Movement Complete!"

    def publish_current_state(self,state):
        self.state_pub.publish(state)

if __name__ == "__main__":
    os.system("clear")
    ShiaStateMachine()
