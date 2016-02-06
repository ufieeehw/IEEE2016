#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int8, Header
from geometry_msgs.msg import Pose, Point32, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3, PointStamped
from sensor_msgs.msg import PointCloud,ChannelFloat32
import tf

import numpy as np

class Block():
    def __init__(self, color, coordinate = 'na'):
        self.color = color
        # This will allow the user to input a point message from ros or a list from python
        try:
            self.coordinate = [coordinate.x,coordinate.y,coordinate.z]
        except:
            self.coordinate = coordinate

    def __repr__(self):
        # How the object prints
        return "%06s" % self.color

class EndEffector():
    def __init__(self, gripper_count):
        self.gripper_count = gripper_count
        
        # Array of the blocks in the gripper - 0 is the left most gripper
        self.block_positions = []
        self.holding = 0
        for i in range(gripper_count):
            b = Block("none")
            self.block_positions.append(b)
    
    def pickup(self, block, gripper_number):
        # Adds the block to gripper specified
        if self.block_positions[gripper_number] == "none":
            self.block_positions[gripper_number] = block
            self.holding += 1
            return True
        else:
            return False

    def which_gripper(self,color):
        # Returns the gripper numbers of the grippers containing the blocks with specified color
        gripper_numbers = []
        for i,b in enumerate(self.block_positions):
            if b.color == color: gripper_numbers.append(i)

        return gripper_numbers

class ProcessBlocks():
    def __init__(self, *end_effectors):
        self.point_sub = rospy.Subscriber("/blocks", PointCloud, self.got_points, queue_size=1)
        self.ee_point_pub = rospy.Publisher("/ee_move", PointStamped, queue_size=1)

        self.blocks = np.array([])
        self.expected_blocks = 16
        
        self.end_effectors = end_effectors

        print "Waiting for message..."

    def got_points(self,msg):
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

    def find_missing_blocks(self):
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

    def pickup_blocks(self):
        # Pick up blocks and keep track of the order they are in
        # # Temp sorted blocks modifier

        # self.blocks_sorted[0][0].color = "none"
        # self.blocks_sorted[0][1].color = "none"
        # self.blocks_sorted[0][2].color = "none"
        # self.blocks_sorted[0][3].color = "none"
        # self.blocks_sorted[0][4].color = "none"
        # self.blocks_sorted[0][5].color = "none"
        # self.blocks_sorted[0][6].color = "none"
        # self.blocks_sorted[0][7].color = "none"

        # One-by-one, move each end effector in position to pick up certain sets of blocks, then pick up blocks and register the locations.
        temp_planner_blocks = self.blocks_sorted
        for e in self.end_effectors:
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
                if valid_counter_top == e.gripper_count:
                    groups[0].append([index - valid_counter_top + 1, valid_counter_top])
                    valid_counter_top = 0
                if valid_counter_bottom == e.gripper_count:
                    groups[1].append([index-valid_counter_bottom + 1,valid_counter_bottom])
                    valid_counter_bottom = 0
                
                index += 1
            groups[0].append([8 - valid_counter_top, valid_counter_top])
            groups[1].append([8 - valid_counter_bottom,valid_counter_bottom])

            # Pick best grouping for end effector
            largest_group = max(groups[0], key=lambda g:g[1])
            waypoint = np.array([0,0,0]).astype(np.float64)

            # Determine the best location to move the end effector
            row = 0
            if largest_group[1] == 0: 
                largest_group = max(groups[1], key=lambda g:g[1])
                row = 1
                if largest_group[1] == 0: print "Nothing left"
            
            
            for b in self.blocks_sorted[row][largest_group[0]:largest_group[0]+largest_group[1]]: 
                waypoint += np.array(b.coordinate)
                
            waypoint /= largest_group[1]
            self.pub_ee_point(waypoint)
            #print waypoint

    def drop_color(self, en):
        pass


    def pub_ee_point(self,point):
        self.ee_point_pub.publish(PointStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="base_link"
                ),
                point=Point32(
                    x=point[0],
                    y=point[1],
                    z=point[2],
                )
            )
        )
        print point

if __name__ == "__main__":
    rospy.init_node('str_command')
    e1 = EndEffector(4)
    e2 = EndEffector(4)
    
    ProcessBlocks(e1,e2)

    rospy.spin()