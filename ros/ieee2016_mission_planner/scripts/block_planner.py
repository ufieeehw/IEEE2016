#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int8, Header
from geometry_msgs.msg import Pose, Point32, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
from sensor_msgs.msg import PointCloud,ChannelFloat32
import tf

import numpy as np

class Block():
    def __init__(self, color, coordinate):
        self.color = color
        # This will allow the user to input a point message from ros or a list from python
        try:
            self.coordinate = [coordinate.x,coordinate.y,coordinate.z]
        except:
            self.coordinate = coordinate

    def print_block(self):
        print "Color:",self.color
        print "x:",self.coordinate[0],"y:",self.coordinate[1],"z:",self.coordinate[2]
        print 

class ProcessBlocks():
    def __init__(self):
        self.point_sub = rospy.Subscriber("/blocks", PointCloud, self.got_points, queue_size=1)

        self.points = np.array([])
        self.expected_blocks = 18

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
            self.points = np.append(self.points,b)

        print "Blocks found."
        # Unsubscribe from the point cloud subscriber and start analysis of points
        self.point_sub.unregister()
        self.find_missing_blocks()

    def find_missing_blocks(self):
        # Find out how many missing blocks there are and identify where they should be
        missing = self.expected_blocks-len(self.points)
        print "Detected",missing,"blocks missing."
        
        # If too many are missing, just redo the intial search
        if missing > 4:
            print "There are too many missing blocks!"
            #self.redetect_blocks()

        if missing == 0:
            # If we found all the codes we wanted, great! Move on to generating an order for the arm
            self.generate_order()
        else:
            # Try to find out which are missing (move this to the gpu later)
            """
            This whole thing is kind of sketchy so beware....
            Start with finding the left most block and check for another block in that column. If there is one, delete them both from the list
            and find the left most block again. As you do this, populate an ordered array of each block in each position (starting from the top left).
            At the end, you'll know which ones are missing and can deal with that.
            """

            # Where to hold the final output
            self.blocks = [[],
                           []]

            # Preset paramters
            variance = .01 #m
            max_expected_dx = .07 #m, used to detect missing cols
            max_expected_dy = .04 #m, used to detect missing row elements

            # These will be set during the loop
            top_row_y = 0 #m
            bottom_row_y = 0 #m
            threshold_y = 0 #m
            last_smallest_x = 0 #m

            # Used for saving output array
            current_col = 0

            while len(self.points) > 0:
                smallest_x_point = Block("none",[1e99,1e99,1e99]) #big number
                for b in self.points:
                    if b.coordinate[0] < smallest_x_point.coordinate[0]: 
                        # Find the left most detected block
                        smallest_x_point = b

                        # If this is the first column then dont do the rest of this block
                        if current_col == 0:
                            break

                        if abs(b.coordinate[0] - last_smallest_x) > max_expected_dx:
                            print "Missing Col"
                            current_col += 1
                        if b.coordinate[1] 

                        

                
                self.points = self.points[self.points != smallest_x_point]#np.delete(self.points, np.where(self.points==smallest_x_point))

                print smallest_x_point.coordinate
                print "========================="
                found_row = False
                for b in self.points:
                    if abs(smallest_x_point.coordinate[0] - b.coordinate[0]) <= max_expected_dy:
                        found_row = True
                        last_smallest_x = b.coordinate[0]
                        self.points = self.points[self.points != b]

                if not found_row:
                    print "Missing Element"



    def generate_order(self):
        # Figure out the best order to go about picking up blocks and where to put them on Shia
        pass

if __name__ == "__main__":
    rospy.init_node('str_command')
    ProcessBlocks()
    rospy.spin()