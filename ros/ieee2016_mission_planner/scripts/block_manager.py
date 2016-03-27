#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool, Int8, Header
from geometry_msgs.msg import Pose, Point32, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3, PointStamped, Point
from sensor_msgs.msg import PointCloud, ChannelFloat32
import tf

roslib.load_manifest('ieee2016_vision')
from camera_manager import Camera
from point_intersector import PointIntersector
from ieee2016_msgs.msg import BlockStamped
from ieee2016_msgs.srv import ArmWaypoint

import numpy as np
from kd_tree import KDTree
import time

class Block():
    def __init__(self, color, coordinate = 'na'):
        self.color = color
        # Coordinates stored here are depreicated
        # This will allow the program to input a point message from ros or a list from python.
        try:
            self.coordinate = [coordinate.x,coordinate.y,coordinate.z]
        except:
            self.coordinate = coordinate

    def __repr__(self):
        # How the object prints
        return "%06s" % self.color

class Gripper():
    '''
    There are multiple grippers on the end of each end effector. This deals with keeping track of the blocks in
    each gripper and the TF frames associated with each gripper.
    '''
    def __init__(self, parent_ee_number, gripper_number, block):
        self.frame_id = str(parent_ee_number) + "-G" + str(gripper_number)
        self.block = block
        self.servo_id = None

    def  __repr__(self):
        return str(self.frame_id + ":" + self.block.color)

    def get_tf(self,tf_listener,from_frame):
        '''
        Given a tf_listener (not defined here so we don't create 8 instances of a tf listner), return [[x,y,z],[rx,ry,rz]]
        between the tf frame 'from_frame' to this grippers frame.
        Used for determining how many blocks we can pick up.
        '''
        time = tf_listener.getLatestCommonTime(from_frame, self.frame_id)
        tf_listener.waitForTransform(from_frame, self.frame_id, time, rospy.Duration(1.0))
        pos, quaternion = tf_listener.lookupTransform(from_frame,self.frame_id, time)
        rot = tf.transformations.euler_from_quaternion(quaternion)

        return np.array([pos,rot])

class EndEffector():
    '''
    The end effector is a grouping of grippers. The end effector object deals with keeping track of the grippers.
    Gripper 0 is the leftmost gripper.
    '''
    def __init__(self, gripper_count, ee_number, cam_position):
        self.gripper_count = gripper_count
        self.frame_id = "EE"+str(ee_number)

        # Array of the blocks in the gripper - 0 is the left most gripper
        self.gripper_positions = []
        self.cam_position = cam_position
        self.holding = 0

        for i in range(gripper_count):
            # Define new gripper and add it to the gripper list
            self.gripper_positions.append( Gripper(ee_number, i, Block("none") ) )

        print self.gripper_positions
    
    def pickup(self, block, gripper):
        # Adds the block to grippers specified. More of a virtual pickup - only adds block to this gripper object.
        g = self.gripper_positions.index(gripper)
        self.gripper_positions[g].block = block
        self.holding += 1

    def drop(self, grippers):
        # Drops blocks in the grippers specified. More of a virtual drop.
        #if len(grippers) == 1: grippers = [grippers]
        print grippers
        for gripper in grippers:
            print gripper
            g = self.gripper_positions.index(gripper)
            self.gripper_positions[g].block = Block("none")
            self.holding -= 1

    def which_gripper(self,color):
        # Returns the gripper numbers of the grippers containing the blocks with specified color
        grippers_to_drop = []
        for g in self.gripper_positions:
            if g.block.color == color: grippers_to_drop.append(g)

        return grippers_to_drop

    def __repr__(self):
        # How the object prints
        return str(self.gripper_positions)

class WaypointGenerator():
    '''
    The arm waypoints generator. (Anything relating to simulation_blocks can be removed for competition)

    Given a kd_tree of points, attempt to find the top left point and generate an arm waypoint there.
    Do this for both arms then return the list back to the caller.

    '''
    def __init__(self, *ee):
        self.ee_pose_pub = rospy.Publisher("/arm/waypoint", PoseStamped, queue_size=1)

        self.ee_list = ee

        self.picked_up = 0

        self.tf_listener = tf.TransformListener()
        print "> Waypoint Generator Online."

    def generate_arm_waypoints(self, block_tree, pickup):
        '''
        Given some block_tree, try to pick up 'pickup' number of blocks with all the end effectors. Starting
        If pickup is -1, we will try to pick up as many blocks as the end effector can.
        The qr_rotation is used when visual servoing up close, we want to know the color and orientation of the block for quick template matching.

        Waypoints are returned as a list [[ee1_base, ee1_base_target_location, grippers_to_actuate, qr_rotation],[ee2_base, ee2_base_target_location, grippers_to_actuate, qr_rotation]]

        This could have problems picking up blocks on the edges of the map. Future self: test and fix that.
        '''

        waypoints_list = []

        # Needed to remove none blocks, just for simulation
        sorted_blocks = []
        for block in block_tree.nodes:
            #print block
            if block.linked_object[0] != "none":
                sorted_blocks.append(block)

        block_tree = self.make_temp_tree(sorted_blocks)

        #print block_tree.nodes
        for ee in self.ee_list:
            if pickup == -1:
                pickup = 0
                for gripper in ee.gripper_positions:
                    if gripper.block.color == "none": pickup += 1

            # Loop through this until our gripper is full.
            while ee.holding < pickup and len(block_tree.nodes) != 0:

                # Sort by order of largest z then smallest x.
                sorted_blocks = sorted(block_tree.nodes, key=lambda node:(-node.point[2],node.point[0]))

                # Find the base gripper.
                for gripper in ee.gripper_positions:
                    if gripper.block.color == "none":
                        base_gripper = gripper
                        grippers_to_actuate = [ee.gripper_positions.index(gripper)]
                        break

                # Find location to move the base gripper to. Then add that block to the gripper.
                target_waypoint = sorted_blocks[0].point
                ee.pickup(Block(sorted_blocks[0].linked_object[0]),base_gripper)
                self.picked_up += 1

                print "Gripper:",base_gripper,"to:",sorted_blocks[0]
                print "Grippers to Actuate:",grippers_to_actuate

                block_tree = self.make_temp_tree(sorted_blocks[1:])

                #print simulation_blocks
                #simulation_blocks[simulation_blocks.index([sorted_blocks[0].point.tolist(),sorted_blocks[0].linked_object[0]])][1] = "none"

                # Loop through the remaining grippers and check if they can pick up and blocks.
                block_tolerance = .0375 # m
                cam_block = None
                for i in range(grippers_to_actuate[0]+1,pickup):
                    # Get the relative y position and check if there is a block within tolerance of that point.
                    rel_gripper_position = abs(ee.gripper_positions[i].get_tf(self.tf_listener,from_frame=base_gripper.frame_id)[0][1])
                    expected_location = target_waypoint+np.array([rel_gripper_position,0,0])

                    closest_block = block_tree.search(expected_location)

                    if closest_block is None: break
                    if closest_block[0] < block_tolerance:
                        #print "Block found! :",closest_block
    
                        # We are going to use this for close up navigation.                        
                        if ee.gripper_positions[i] == ee.cam_position:
                            cam_block = closest_block[1]

                        # Add that block to the gripper, assume we will pick it up soon. 
                        ee.pickup(Block(closest_block[1].linked_object[0]),ee.gripper_positions[i])
                        grippers_to_actuate.append(i)
                        self.picked_up += 1

                        #simulation_blocks[simulation_blocks.index([closest_block[1].point.tolist(),closest_block[1].linked_object[0]])][1] = "none"

                        # Now that we have saved it to the gripper, remove it and search for the next gripper block.
                        block_tree.nodes.remove(closest_block[1])
                        block_tree = self.make_temp_tree(block_tree.nodes)
                    else:
                        # If theres no block adjacent to the previous gripper, then we dont want to continue
                        break

                qr_rotation = None
                if cam_block:
                    qr_rotation = cam_block.linked_object[1]
                waypoints_list.append([base_gripper.frame_id, target_waypoint, grippers_to_actuate, qr_rotation])

            # We have added all the contiuous blocks starting from the top left, now if there is still space left in the gripper
            # see if we can fit any more blocks onto the gripper.

        # Returning the new tree, the waypoints for the gripper, and the temp simulation block list
        return block_tree, waypoints_list, simulation_blocks

    def make_temp_tree(self, new_list):
        '''
        Generates a new tree with the new list. List elements should be Node objects.
        Move this to KDTree at some point
        '''
        temp_tree = KDTree()
        for l in new_list:
            temp_tree.insert(l.point,linked_object=l.linked_object)
        return temp_tree

class BlockFitter():
    '''
    NOTE: All measurements are in meters.
    '''
    def __init__(self):
        self.point_publisher = rospy.Publisher("/estimated_block_positions", PointCloud, queue_size=5)

        self.dx = .0635
        self.dy = .0635 
        self.dz = .0381 
        self.base_y = 2.174

    def construct_zone_frame(self, stage_letter):
        '''
        Build an empty frame to populate the blocks into
        Stage A is the middle height blue only block area.
        Stage B is the tallest multicolor multisize block zone.
        Stage C is the shortest multicolor block zone.
        '''
        self.blocks = []

        if stage_letter == 'A':
            base_x = 0.373 #abritrary right now
            base_z = 0.1778 
        if stage_letter == 'B':
            base_x = 1.008 #abritrary right now
            base_z = 0.254
        if stage_letter == 'C':
            base_x = 1.9304 #abritrary right now
            base_z = 0.127

        for row in range(2):
            for col in range(8):
                block_x = base_x + col * self.dx
                block_y = self.base_y
                block_z = base_z + row * self.dz
                self.blocks.append([block_x,block_y,block_z])

        self.publish_points()

    def correct_position(self, block_index, measured_x):
        '''
        When we register a block with a camera, update the entire block array position to match with that data.
        '''
        return

    def publish_points(self):
        points = []
        channels = [[],[],[]]
        # for p in self.blocks:
        #     #print p.linked_object
        #     if p.linked_object[] == "blue":
        #         channels[0].append(0) #R
        #         channels[1].append(0) #G
        #         channels[2].append(1) #B
        #     elif p.linked_object  == "red":
        #         channels[0].append(1) #R
        #         channels[1].append(0) #G
        #         channels[2].append(0) #B
        #     elif p.linked_object  == "green":
        #         channels[0].append(0) #R
        #         channels[1].append(1) #G
        #         channels[2].append(0) #B
        #     elif p.linked_object  == "yellow":
        #         channels[0].append(1) #R
        #         channels[1].append(1) #G
        #         channels[2].append(0) #B
        #     elif p.linked_object  == "none":
        #         channels[0].append(0) #R
        #         channels[1].append(0) #G
        #         channels[2].append(0) #B

        #     points.append(Point32(*p.point))

        #rgb_channels = [ChannelFloat32(name="r", values=channels[0]),ChannelFloat32(name="g", values=channels[1]),ChannelFloat32(name="b", values=channels[2])]
        time.sleep(.25)
        self.point_publisher.publish(PointCloud(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                    ),
                points=points,
                #channels=rgb_channels
            )
        )


class BlockServer():
    '''
    BlockServer acts as a server to deal with block detection from the cameras. 

    Given a BlockStamped message, the server will find the actual point in the map frame and keep an updated pointcloud as Shia moves.
    '''
    def __init__(self, *cameras):
        self.point_pub = rospy.Publisher("/block_test", PointStamped, queue_size=10)
        rospy.Subscriber("/camera/block_detection", BlockStamped, self.got_block, queue_size=1)

        self.cameras = cameras

        # Make kd-tree with a tolerance when trying to add duplicated blocks
        self.k = KDTree(.02) #m
        self.intersector = PointIntersector()

    def got_block(self,msg):
        # Find the camera that this image was taken in and transform points appropriately.
        camera = [c for c in self.cameras if c.name == msg.header.frame_id][0]
        map_point = self.intersector.intersect_point(camera, msg.point, offset=msg.offset)
        self.publish_point(map_point)
        #try:
        self.k.insert_unique_average(map_point,[msg.color, msg.rotation_index])
        # except:

        #     rospy.logwarn("An ERROR was found and excepted.")

        print self.k
        print

    def publish_point(self,point):
        
        p = Point(x=point[0], y=point[1], z=point[2])
        h = Header(stamp=rospy.Time.now(), frame_id="map")
        self.point_pub.publish(header=h, point=p)



if __name__ == "__main__":
    rospy.init_node('block_manager')
    b = BlockFitter()
    b.construct_zone_frame('A')
    # ee1 = EndEffector(gripper_count=4, ee_number=1, cam_position=1)
    # ProcessBlocks(ee1)
    rospy.spin()