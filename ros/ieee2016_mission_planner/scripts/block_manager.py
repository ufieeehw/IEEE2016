#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool, Int8, Header
from geometry_msgs.msg import Pose, Point32, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3, PointStamped, Point
from sensor_msgs.msg import PointCloud,ChannelFloat32
import tf

roslib.load_manifest('ieee2016_vision')
from camera_manager import Camera
from point_intersector import PointIntersector
from ieee2016_msgs.msg import BlockStamped
from ieee2016_msgs.srv import ArmWaypoint

import numpy as np
from kd_tree import KDTree

class Block():
    def __init__(self, color, coordinate = 'na'):
        self.color = color
        # This will allow the program to input a point message from ros or a list from python
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

    def  __repr__(self):
        return self.frame_id

class EndEffector():
    '''
    The end effector is a grouping of grippers. The end effector object deals with keeping track of the grippers.
    Gripper 0 is the gripper leftmost gripper.
    '''
    def __init__(self, gripper_count, ee_number, cam_position):
        self.gripper_count = gripper_count
        self.frame_id = "EE"+str(ee_number)

        # Array of the blocks in the gripper - 0 is the left most gripper
        self.block_positions = []
        self.cam_position = cam_position
        self.holding = 0

        for i in range(gripper_count):
            # Define new gripper and add it to the gripper list
            self.block_positions.append( Gripper(ee_number, i, Block("none") ) )

        print self.block_positions
    
    def pickup(self, block, *gripper_number):
        # Adds the block to gripper specified
        for g in gripper_number:
            if self.block_positions[g] == "none":
                self.block_positions[g] = block
                self.holding += 1
                return True
            else:
                return False

    def which_gripper(self,color):
        # Returns the gripper numbers of the grippers containing the blocks with specified color
        gripper_numbers = []
        for i,b in enumerate(self.block_positions):
            if b.color == color: block_positions.append(i)

        return gripper_numbers

    def __repr__(self):
        # How the object prints
        return str(self.block_positions)


class ProcessBlocks():
    '''
    Take detected blocks from a kd-tree populated with blocks, try to find missing blocks, generate a position for the 
    grippers to move to based on how many blocks are left, then deal with dropping off the blocks into the appropriate bins.

    This needs to be modified to work in all cases, with half blocks namely.
    '''
    def __init__(self, *end_effectors):
        self.ee_pose_pub = rospy.Publisher("/arm/waypoint", PoseStamped, queue_size=1)
        #self.point_sub = rospy.Subscriber("/camera/block_point_cloud", PointCloud, self.got_points, queue_size=1)

        self.move_arm = rospy.ServiceProxy('/robot/arms/set_waypoint', ArmWaypoint)

        self.expected_blocks = 16
        
        self.end_effectors = end_effectors

        print "Waiting for message..."

    def got_points(self,msg):
        '''
        Depreciated, now we are just passing a kd_tree of blocks in rather then sending them as a ros point cloud.
        '''

        #if len(self.points) < 1:
        #print msg.channels[0].values
        for i,p in enumerate(msg.points):
            color_rgb = (msg.channels[0].values[i],msg.channels[1].values[i],msg.channels[2].values[i])

            # Colors are given as floats for each channel: RGB
            if color_rgb == (1.,0.,0.):
                color = "red"
            elif color_rgb == (0.,1.,0.):
                color = "green"
            elif color_rgb == (0.,0.,1.):
                color = "blue"
            elif color_rgb == (1.,1.,0.):
                color = "yellow"
            elif color_rgb == (1.,1.,1.):
                color = "white"
            else:
                color = "none"

            b = Block(color,p)
            self.blocks = np.append(self.blocks,b)

        print "Blocks found."
        # Unsubscribe from the point cloud subscriber and start analysis of points
        self.point_sub.unregister()
        self.find_missing_blocks()

    def find_missing_blocks(self, block_tree):
        # Go through the tree we were given and extract block information.
        self.blocks = np.array([])
        for b in block_tree:
            block = Block(b[1].point,b[1].linked_object)
            self.blocks = np.append(self.blocks,block)

        # Find out how many missing blocks there are and identify where they should be
        missing = self.expected_blocks-len(self.blocks)
        print "Detected",missing,"blocks missing."
        
        # If too many are missing, just redo the intial search
        if missing > 4:
            print "There are too many missing blocks!"
            #self.redetect_blocks()

        # Try to find out which are missing (move this to the gpu later)
        """
        This whole thing is kind of sketchy so beware....
        Sort blocks in order of x position. Then go through and detect if each pair of detected points are in the same x column.
        Then find which of the two is on top and which is on bottom. Add blocks to the sorted list appropriately.

        There are a lot of functions to catch extranious cases, and it should be tested with as many cases as possible.
        """

        # Where to hold the final output
        blocks_sorted = [[[],[],[],[],[],[],[],[]],
                         [[],[],[],[],[],[],[],[]]]

        # Preset paramters
        variance = .05 #m
        max_expected_dx = .075 #m, used to detect missing columns
        normal_dx = .0635 #m, used to fill in missing columns
        normal_dz = .0381 #m, used to fill in missing row elements
        
        # Calculate the threshold for a top block or a bottom block, will be used for columns with missing elements
        threshold_z = (max(self.blocks, key=lambda b: b.coordinate[2]).coordinate[2] + 
                       min(self.blocks, key=lambda b: b.coordinate[2]).coordinate[2])/2.0

        # This assumes the first blocks are both there, fix it so that doesnt have to be an assumption
        # It'll be alittle faster to use a local copy of the array
        blocks = sorted(self.blocks,key=lambda b: b.coordinate[0])
        for i in range(self.expected_blocks/2):
            # Check if we are at the end of the block list
            if i*2 >= len(blocks): 
                print "Error, first or last columns missing."
                break

            if i*2 == len(blocks)-1: 
                if blocks[i*2].coordinate[2] > threshold_z:
                    # missing block is a bottom block
                    block = Block("none",[blocks[i*2-1].coordinate[0]+normal_dx,
                                          blocks[i*2-1].coordinate[1],
                                          blocks_sorted[1][i-1].coordinate[2]])
                    blocks.insert(2*i+1, block)
                else:
                    # missing block is a top block
                    block = Block("none",[blocks[i*2-1].coordinate[0]+normal_dx,
                                          blocks[i*2-1].coordinate[1],
                                          blocks_sorted[0][i-1].coordinate[2]])
                    blocks.insert(2*i+1, block)
            
            # Check to see if a whole column is missing
            if abs(blocks[2*i].coordinate[0] - blocks[2*i-1].coordinate[0]) > max_expected_dx and i is not 0:
                top_block = Block("none",[blocks[i*2-1].coordinate[0]+normal_dx,
                                          blocks[i*2-1].coordinate[1],
                                          blocks_sorted[0][i-1].coordinate[2]])
                bot_block = Block("none",[blocks[i*2-1].coordinate[0]+normal_dx,
                                          blocks[i*2-1].coordinate[1],
                                          blocks_sorted[1][i-1].coordinate[2]])
                blocks.insert(2*i, top_block)
                blocks.insert(2*i+1, bot_block)

            # Make sure the two blocks are in the same column
            if abs(blocks[i*2].coordinate[0] - blocks[i*2+1].coordinate[0]) < variance:                
                if blocks[i*2].coordinate[2] > blocks[i*2+1].coordinate[2]:
                    blocks_sorted[0][i] = blocks[i*2]
                    blocks_sorted[1][i] = blocks[i*2+1]
                else:
                    blocks_sorted[0][i] = blocks[i*2+1]
                    blocks_sorted[1][i] = blocks[i*2]
            else:
                # Handles if any arbitrary element is missing
                if blocks[i*2].coordinate[2] > threshold_z:
                    # the missing block is a bottom block
                    blocks_sorted[0][i] = blocks[2*i]
                    blank = Block("none", [blocks[i*2].coordinate[0],
                                           blocks[i*2].coordinate[1],
                                           blocks[i*2].coordinate[2]-normal_dz])
                    blocks.insert(2*i+1, blank)
                    blocks_sorted[1][i] = blocks[2*i+1]
                else:
                    # the missing block is a top block
                    blocks_sorted[1][i] = blocks[2*i]
                    blank = Block("none", [blocks[i*2].coordinate[0],
                                           blocks[i*2].coordinate[1],
                                           blocks[i*2].coordinate[2]+normal_dz])
                    blocks.insert(2*i+1, blank)
                    blocks_sorted[0][i] = blocks[2*i+1]
                
        # Just for debugging
        print "Detected Blocks:"
        for b in blocks_sorted:
            print b
        print
        # Make class wide copy of the sorted blocks (pickup_blocks should be called from the main program)
        self.blocks_sorted = blocks_sorted
        self.pickup_blocks()

    def make_arm_waypoints(self):
        # Pick up blocks and keep track of the order they are in

        arm_waypoints = []

        # One-by-one, move each end effector in position to pick up certain sets of blocks, then pick up blocks and register the locations.
        temp_planner_blocks = self.blocks_sorted
        for ee in self.end_effectors:
            # Find the group of blocks to pick up. We're looking for the largest group of topmost blocks
            valid_counter_top = 0
            valid_counter_bottom = 0
            # Holds block groupings for top and bottom as [ending_index,length]
            groups = [[],[]]
            index = 0
            for upper_b,lower_b in zip(self.blocks_sorted[0],self.blocks_sorted[1]):
                if upper_b.color != "none":
                    valid_counter_top += 1 

                    # If there is a block on top, that means that there isnt one on the bottom 
                    groups[1].append([index-valid_counter_bottom,valid_counter_bottom])
                    valid_counter_bottom = 0
                else:
                    # Check for bottom row blocks same way as top
                    if lower_b.color != "none": 
                        valid_counter_bottom += 1
                    else:
                        groups[1].append([index-valid_counter_bottom,valid_counter_bottom])
                        valid_counter_bottom = 0

                    groups[0].append([index - valid_counter_top, valid_counter_top])
                    valid_counter_top = 0
                
                # Save max sized groupings
                if valid_counter_top == ee.gripper_count:
                    groups[0].append([index - valid_counter_top + 1, valid_counter_top])
                    valid_counter_top = 0
                if valid_counter_bottom == ee.gripper_count:
                    groups[1].append([index-valid_counter_bottom + 1,valid_counter_bottom])
                    valid_counter_bottom = 0
                
                index += 1
            groups[0].append([8 - valid_counter_top, valid_counter_top])
            groups[1].append([8 - valid_counter_bottom,valid_counter_bottom])

            # Pick best grouping for end effector
            largest_group = max(groups[0], key=lambda g:g[1])
            waypoint = np.array([0,0,0]).astype(np.float64)

            # Should we pick up blocks in row 0 or row 1
            row = 0
            if largest_group[1] == 0: 
                largest_group = max(groups[1], key=lambda g:g[1])
                row = 1
                if largest_group[1] == 0: print "Nothing left"
            

            # The waypoint is set so that the gripper with the camera above it will move to
            # the grouping of blocks. This may not be nessicary, but it can't hurt.
            waypoint = self.blocks_sorted[row][largest_group[0]+1].coordinate
            # We dont want the point to be exactly on the block, instead we want it a safe buffer
            # distance out infront of the block.
            buffer_distance = .2 #m
            waypoint -= np.array([0,buffer_distance,0])

            # Even though the gripper isnt actually holding the block, we assume it will be
            self.set_gripper_blocks(self.blocks_sorted[row][largest_group[0]:largest_group[0]+largest_group[1]])

            arm_waypoints.append([ee.gripper_positions[ee.cam_position],waypoint])

        return arm_waypoints

    def set_gripper_blocks(self, ee, blocks):
        for gripper,block in zip(ee.block_positions,blocks):
            gripper.block = block

    def drop_color(self, ee):
        pass

    def pub_ee_pose(self,gripper,point):
        q = tf.transformations.quaternion_from_euler(0, 0, 1.5707)
        pose_stamped = PoseStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id="map"
                    ),
                pose=Pose(
                        position=Point(*point),
                        orientation=Quaternion(*q)
                    )
            )
        print point
        self.ee_pose_pub.publish(pose_stamped)
        self.move_arm(str(gripper),pose_stamped)

class BlockServer():
    '''
    BlockServer acts as a server to deal with block detection from the cameras. 

    Given a BlockStamped message, the server will find the actual point in the map frame and keep an updated pointcloud as Shia moves.
    '''
    def __init__(self, *cameras):
        rospy.Subscriber("/camera/block_detection", BlockStamped, self.got_block, queue_size=20)
        self.cameras = cameras

        # Make kd-tree with a tolerance when trying to add duplicated blocks
        self.k = KDTree(.03175) #m
        self.intersector = PointIntersector()

    def got_block(self,msg):
        # Find the camera that this image was taken in and transform points appropriately.
        camera = [c for c in self.cameras if c.name == msg.header.frame_id][0]
        map_point = self.intersector.intersect_point(camera, msg.point, time=msg.header.stamp)
        self.k.insert_unique(map_point,msg.color)

if __name__ == "__main__":
    rospy.init_node('block_manager')
    # c1 = Camera("cam_1")
    # c1.activate()
    # b_s = BlockServer(c1)
    ee1 = EndEffector(gripper_count=4, ee_number=1, cam_position=1)
    ProcessBlocks(ee1)
    rospy.spin()