#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool, Int8, Header
from sensor_msgs.msg import PointCloud, ChannelFloat32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3, Point32, PointStamped, PoseWithCovarianceStamped, PoseWithCovariance
from ieee2016_msgs.srv import NavWaypoint, ArmWaypoint, RequestMap, DynamixelControl
from ieee2016_msgs.msg import StartNavigation
from rospy.numpy_msg import numpy_msg
from robot_localization.srv import SetPose
import tf

roslib.load_manifest('ieee2016_vision')
from camera_manager import Camera
from arm_controller import ArmController, ServoController
from block_manager import EndEffector, BlockServerV2, WaypointGenerator
from waypoint_utils import WaypointServer
from qr_detector import DetectQRCodeTemplateMethod


# Temp Imports
roslib.load_manifest('ieee2016_simulator')
import random

from kd_tree import KDTree
import numpy as np
import time
import os

class ShiaStateMachine():
    '''
    This is the main program that will run when the start button is pressed.
    This is still a work in progress.
    '''
    def __init__(self):
        self.ros_manager = RosManager(self)

        self.current_state = 0
        self.map_version = 0

        self.running = False

        print "> ========= Creating Limbs ========="
        # Defining Shia's limbs. 
        # 1 is on the left, 2 is on the right
        self.ee1 = EndEffector(gripper_count=4, ee_number=1, cam_position=2)
        self.ee2 = EndEffector(gripper_count=4, ee_number=2, cam_position=2)
        self.ee_list = [self.ee1,self.ee2]

        self.cam1 = Camera(cam_number=1)
        self.cam2 = Camera(cam_number=2)

        print "> ========= Creating Detection Objects ========="
        # Objects for detecting and returning location of QR codes. Parameters are the set distances from the codes (cm).
        self.qr_distances = [50]
        #self.qr_detector = DetectQRCodeTemplateMethod(self.qr_distances)
        self.waypoint_generator = WaypointGenerator(self.ee1, self.ee2)
        #self.arm_controller = ArmController()

        # Misc Objects
        self.point_cloud_generator = temp_GenerateBlockPoints(16)
        self.point_cloud_generator.generate_b_blocks()
        print
        print "> ========= State Machine Init Complete ========="
        print "> Waiting for map selection and start command."

        rospy.spin()

    def load_waypoints(self):
        waypoints_temp = load_waypoints()
        if self.map_version == 2:
            # If we are on the right side map configuration, switch all the waypoints 
            for point in waypoints_temp:
                waypoints_temp[key][0] = self.ros_manager.far_wall_x - waypoints_temp[key][0]  
        self.waypoints = waypoints_temp

    def begin_2(self):
        '''
        This will run the map configuration where we start on the left side of the map.
        '''
        print
        print "> Running"
        print "> Map Version:",self.map_version
        print
        self.load_waypoints()
        self.train_box_processor = temp_ProcessTrainBoxes(self.waypoints)

        self.current_state+=0

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
                self.ros_manager.set_nav_waypoint(self.waypoints['through_box'], )
                train_box_colors = self.train_box_processor.process_train_blocks()
                time.sleep(1)
                print train_box_colors

                self.current_state += 1

            elif self.current_state == 2:
                '''
                The goals of this state are:
                    1. Nove to and begin processing B blocks.
                    2. Move to the next B blocks processing location and continue processing.
                    3. Continue going to B block waypoints until all blocks are processed.
                    4. Generate Arm Waypoints
                '''
                print "=== STATE 2 ==="
                self.ros_manager.set_nav_waypoint(self.waypoints['process_b_1'])
                b_tree = self.point_cloud_generator.generate_b_camera_view()
                b_blocks = self.point_cloud_generator.b_blocks.tolist()
                b_tree, arm_waypoints, b_blocks = self.waypoint_generator.generate_arm_waypoints(b_tree, -1, b_blocks)
                print "> Waypoints Generated."
                #self.ros_manager.set_nav_waypoint(self.waypoints['process_b_2'])

                self.current_state += 1

            elif self.current_state == 3:
                '''
                The goals of this state are:
                    1. Move to pick up blocks with first end effector
                    2. Do this again for the second end effector

                When we move in close, we are moving to an esimated position of a block. When we get in close, we try again to identify qr codes
                but this time they will be more accurate. We only want the one qr code infront of the camera.
                '''
                print "=== STATE 3 ==="
                print "> Executing waypoints."

                # Used to determine if we need to rotate or naw.
                last_ee = None
                # Go through each waypoint and make sure we dont hit anything
                for gripper, waypoint, grippers_to_acutate, qr_rotation in arm_waypoints:
                    # We don't want to move and rotate here (unless we arent rotating) - so just rotate and get lined up away from the wall.
                    # Should move this to be part of the controller, but for now it can just go here.
                    this_ee = [ee for ee in self.ee_list if ee.frame_id == ("EE"+gripper[:1])]
                    if this_ee != last_ee:
                        # We need to back up slighly in order to rotate
                        backup_movement = np.copy(self.ros_manager.pose)
                        backup_movement[1] = self.ros_manager.block_wall_y - self.qr_distances[0] #m
                        self.ros_manager.set_nav_waypoint(backup_movement)

                        # Then rotate in place
                        rotate_waypoint = np.copy(waypoint)
                        rotate_waypoint[1] = self.ros_manager.block_wall_y - self.qr_distances[0] #m
                        print "Lining Up"
                        self.ros_manager.set_arm_waypoint(gripper,rotate_waypoint)

                    # Now move in close to the wall. We move so that the edge of the robot is some distance from the wall.
                    # .1524m is 6 inches, or half the robot size
                    distance_off_wall = .0116 #m
                    base_link_to_edge = .1524 #m
                    print "Moving Close"
                    waypoint[1] = self.ros_manager.block_wall_y - (base_link_to_edge + distance_off_wall)
                    self.ros_manager.set_arm_waypoint(gripper,waypoint)

                    # Move elevator to est height

                    # Get a more accurate position estimate where to put the gripper
                    if qr_rotation:
                        print "Correcting Position"
                        camera_gripper = this_ee.gripper_positions[this_ee.cam_position]
                        updated_position = self.qr_detector.visual_servo((base_link_to_edge + distance_off_wall), camera_gripper.block.color, qr_rotation)
                        waypoint[1] = np.copy(self.ros_manager.pose)[1]
                        # We want to line the camera gripper up with the block in the frame
                        self.ros_manager.set_arm_waypoint(camera_gripper, updated_position)


                    # Move dynamixle in

                    # Pick up the blocks IRL somehow
                    print "Picking up with:",grippers_to_acutate

                    # Pull ee back in

                    time.sleep(1)

                    # Used to determine if we need to rotate or naw.
                    last_ee = this_ee

                # Regenerate point cloud - for simulation only
                self.point_cloud_generator.publish_points(b_tree,self.point_cloud_generator.point_cloud_pub)
                #self.point_cloud_generator.b_blocks = np.array(b_blocks, dtype=object)
                #self.point_cloud_generator.generate_b_camera_view()

                self.current_state += 1

            elif self.current_state == 4:
                '''
                The goals of this state are:
                    1. Move to train box_4.
                    2. Release that color on the first end effector.
                    3. Move to next box with a color that we have. Repeat until at the last box.
                    4. Turn around and repeat with the other end effector - starting from box 1.
                    5. If there are still blocks we haven't gotten, move back to reprocess B blocks.
                '''
                print "=== STATE 4 ==="
                # We are going to iterate over 2 directions and to the 4 boxes. The first direction drops off ee1 and the second drops off ee2.
                # Right now we have to stop at each box, it would be good if we didnt have to.
                print "Blocks:",self.ee1
                print "Blocks:",self.ee2
                for direction,ee in enumerate(self.ee_list):
                    # First make sure the gripper has blocks in it.
                    if ee.holding == 0:
                        continue
                    # When direction is 0, we will be pointing one way, when it is 1, we will rotate to face the other way.
                    rotational_offset = direction*np.pi
                    print rotational_offset
                    box_number = 1
                    while box_number <= 4:
                        # Run through before we move and make sure the next box is the color of a block we are holding
                        holding_color = False
                        while not holding_color:
                            if direction == 0:
                                box = 'box_'+str(5-box_number)
                            else:
                                box = 'box_'+str(box_number)

                            grippers_to_drop = ee.which_gripper(train_box_colors[box])
                            if len(grippers_to_drop) is not 0:
                                # If there are blocks that match the color of the box, we can break from this loop and go to that box.
                                holding_color = True
                                break
                            print "No %s blocks detected - skipping that box."%(train_box_colors[box])
                            box_number += 1

                        print "This color:",train_box_colors[box],box
                        # Where should we go and what direction should we point
                        curr_waypoint = np.copy(self.waypoints[box])
                        curr_waypoint[2] += rotational_offset

                        # Move and then when we get there drop off the associated color blocks
                        self.ros_manager.set_nav_waypoint(curr_waypoint)
                        
                        print "Dropping blocks grippers:",grippers_to_drop
                        ee.drop(grippers_to_drop)
                        time.sleep(1)
                        if ee.holding == 0:
                            print "Empty gripper!"
                            break
                        box_number += 1

                if self.waypoint_generator.picked_up == 20:
                    self.waypoint_generator.picked_up = 0
                    self.current_state+=1
                else:
                    # Go back and reprocess B blocks
                    self.current_state = 2

            elif self.current_state == 5:
                '''
                The goals of this state are:
                    1. Move to and detect A blocks.
                    2. Generate arm waypoints.
                '''
                print "=== STATE 5 ==="
                
            elif self.current_state == 6:
                '''
                The goals of this state are:
                    1. Pick up blue blocks with EE1.
                    2. Pick up blue blocks with EE2.
                '''
                print "=== STATE 6 ==="

            elif self.current_state == 7:
                '''
                The goals of this state are:
                    1. Move to boat location.
                    2. Drop all blocks in EE1.
                    3. Rotate 180 degrees and drop off blocks in EE2.
                    4. Possibly go back to state 5.
                '''
                print "=== STATE 7 ==="
       


            else:
                break

            #if self.current_state > 50: self.current_state = 0
            rate.sleep()

class ControlUnit():
    '''
    Deals with keeping track of which stage we are in and what we are doing in that stage. 

    The goal is not to clutter up the state machine with methods that don't need to be there but can instead be here.

    I don't know if this is the best way to implement this.
    '''
    def __init__(self, r, waypoints):
        self.waypoints = waypoints
        self.ros_manager = r
        self.current_block_zone = 'b'
        self.processing_waypoint = None
        self.pickup_stage = 1

    def get_pickup_stage(self, stall = False, set_stage = None):
        '''
        Tells the main program what size blocks to look for or whether to look for top or bottom blocks.
        If stall is True, it wont increment to the next stage when called, otherwise it will autoincrement.

        Returns [qr_distance,top]. Top will be True if we need to look for top blocks. 

        Here are the returns for different stages:
            b)  1st call => [29,True]
                2nd call => [29,True]

                3rd call => [35.25,True]
                4th call => [35.25,True]
                
                5th call => [29,False]
                6th call => [29,False]
                
                7th call => [35.25,False]
                8th call => [35.25,False]
            
            c)  1st call => [29,True]
                2nd call => [29,True]
                
                3nd call => [29,False]
                4nd call => [29,False]
        '''
        full_distance = 29     #cm
        half_distance = 35.25  #cm
        
        if self.current_block_zone == 'b':
            if   self.pickup_stage == 1: ret = [full_distance,True]
            elif self.pickup_stage == 2: ret = [full_distance,True]
            elif self.pickup_stage == 3: ret = [half_distance,True]
            elif self.pickup_stage == 4: ret = [half_distance,True]
            elif self.pickup_stage == 5: ret = [full_distance,False]
            elif self.pickup_stage == 6: ret = [full_distance,False]
            elif self.pickup_stage == 7: ret = [half_distance,False]
            elif self.pickup_stage == 8: 
                ret = [half_distance,False]
                self.pickup_stage = 0

        elif self.current_block_zone == 'c':
            if   self.pickup_stage == 1: ret = [full_distance,True]
            elif self.pickup_stage == 2: ret = [full_distance,True]
            elif self.pickup_stage == 3: ret = [full_distance,False]
            elif self.pickup_stage == 4:
                ret = [full_distance,False]
                self.pickup_stage = 0

        if not stall:
            if set_stage >= 0:
                self.pickup_stage = set_stage
                return ret

            self.pickup_stage += 1

        return ret 

    def get_height_of_blocks(self, camera=False):
        a_height = .1778  #m
        b_height = .254   #m
        c_height = .127   #m
        block_size = .0381 #m

        height = 0
        if not camera:
            height = block_size/2.0 #m (due to the fact the middle of the block is at some height)        
            # Check if we are working with the top blocks and add that height.
            if get_pickup_stage(stall=True)[1]:
                height += block_size

        if self.current_block_zone == 'a': height += a_height
        if self.current_block_zone == 'b': height += b_height
        if self.current_block_zone == 'c': height += c_height

        return height

    def get_distance_to_gripping_position(self, dist_to_wall):
        '''
        Get the distance to move the end effector out in order to grab the block in the right place.

        In Zone B, we know that if we are in stages 3 or 7, we are gripping full blocks (stage 3 as opposed to stage 2 since the stage
            will be incremented after the 2nd qr detection). When we are in stage 5 or 0, that means we just checked for half
            blocks and therefore should be grabbing back half blocks.
        '''
        full_block = .0635 #m
        half_block = .0635/2.0 #m
        if self.current_block_zone == 'a': dist_to_wall += full_block
        if self.current_block_zone == 'b':
            if self.pickup_stage == 3 or self.pickup_stage == 7:
                dist_to_wall += full_block
            else:
                dist_to_wall += 3*half_block

        if self.current_block_zone == 'c': dist_to_wall += full_block


    def get_vision_waypoint(self, ee_number):
        waypoint_name = "vision_%s_%i"%(self.current_block_stage, ee_number)
        waypoint = [value for key in self.waypoints.iteritems() if key.startswith(waypoint_name)]

        return waypoint

    def get_block_waypoint(self, ee_number, gripper_number, block_number):
        '''
        We want to move a gripper on one of the end effectors to a certain block
        Block positions are:
            00  01  02  03  04  05  06  07
            08  09  10  11  12  13  14  15
        '''
        if block_number >= 8: block_number -= 8
        actual_block = block_number - gripper_number
        waypoint_name = "pickup_%s_ee%i_block%i"%(self.current_block_stage, ee_number, actual_block)
        waypoint = [value for key in self.waypoints.iteritems() if key.startswith(waypoint_name)]
        return waypoint

    def next_stage(self):
        if self.current_block_zone == 'a':
            self.current_block_zone = 'c'
            self.current_block_stage = 1
        elif self.current_block_zone == 'b':
            self.current_block_zone = 'a'
            self.current_block_stage = 1
        elif self.current_block_zone == 'c':
            self.current_block_zone = None

class TestingStateMachine():
    def __init__(self):
        self.ros_manager = RosManager(self)

        self.current_state = 0
        self.map_version = 0

        self.running = False

        print "> ========= Creating Limbs ========="
        # Defining Shia's limbs. 
        # 1 is on the left, 2 is on the right
        self.ee2 = EndEffector(gripper_count=4, ee_number=2, cam_position=2)
        self.ee_list = [self.ee2]

        self.cam1 = Camera(cam_number=1)
        self.cam2 = Camera(cam_number=2)
        self.cam_list = [self.cam1,self.cam2]

        print "> ========= Creating Detection Objects ========="
        # Each section has a different block server to listen to blocks.
        self.bs_b_1 = BlockServerV2(self.cam1)
        self.bs_b_2 = BlockServerV2(self.cam2)
        self.bs_c_1 = BlockServerV2(self.cam1)
        self.bs_c_2 = BlockServerV2(self.cam2)
        
        # Objects for detecting and returning location of QR codes. Parameters are the set distances from the codes (cm).
        self.waypoint_generator = WaypointGenerator(self.ee1, self.ee2)
        self.servo_controller = ServoController(self.ee1, self.ee2)

        #self.point_cloud_generator = temp_GenerateBlockPoints(16)
        #self.point_cloud_generator.generate_b_blocks()
        print
        print "> ========= State Machine Init Complete ========="
        print "> Waiting for map selection and start command."

        rospy.spin()

    def begin(self):
        print
        print "> Running Test"
        print "> Map Version:",self.map_version
        print
        # These objects need the map version to work.
        self.w = WaypointServer()
        self.waypoints = self.w.load_waypoints()
        self.control = ControlUnit(self.ros_manager, self.waypoints) # Note that Ken does not have any self.control.
        self.qr_distances = [29]#,35.25
        self.qr_detector = DetectQRCodeTemplateMethod(self.qr_distances)

        time.sleep(5)
        #self.current_state += 1

        self.running = True
        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            
            self.ros_manager.publish_current_state(self.current_state)

            if self.current_state == 1:
                '''
                The goals of this state are:
                    1. Move to and look for the top left 4 blocks with the first camera.
                    2. Get the colors that the grippers on ee1 will pick up.
                    3. Move to and look for the top right 4 blocks with the first camera.
                    4. Get the colors that the grippers on ee2 will pickup.
                    5. Generate waypoints
                '''
                print "=== STATE 1 ==="

                # For EE1
                waypoint = self.control.get_vision_waypoint(ee_number=1)
                self.ros_manager.set_nav_waypoint(waypoint)
                
                self.ros_manager.ultrasonic_side_pub.publish(String(data="left"))
                
                self.cam1.activate()
                pickup_parameters = self.control.get_pickup_stage()
                self.bs_b_1.active = True
                self.qr_detector.normal_scan(self.cam1, "inital_scan", pickup_parameters[0])
                colors_b_1_top = self.bs_b_1.get_block_locations(top=True)[4:]
                colors_b_1_bot = self.bs_b_1.get_block_locations(top=False)[4:]
                self.cam1.deactivate()

                print "> EE1 Colors Generated."

                # For EE2
                waypoint = self.control.get_vision_waypoint(ee_number=2, vel_profile=1)
                self.ros_manager.set_nav_waypoint(waypoint)

                self.cam1.activate()
                pickup_parameters = self.control.get_pickup_stage()
                self.bs_b_2.active = True
                self.qr_detector.normal_scan(self.cam1, "inital_scan", pickup_parameters[0])
                colors_b_2_top = self.bs_b_2.get_block_locations(top=True)[-4:]
                colors_b_2_bot = self.bs_b_2.get_block_locations(top=False)[-4:]
                self.cam1.deactivate()

                print "> EE2 Colors Generated."

                arm_waypoints = self.waypoint_generator.generate_arm_waypointsV2([colors_b_1_top ,colors_b_2_top,colors_b_1_bot ,colors_b_2_bot])
                self.ee_list = self.waypoint_generator.ee_list

                print "> Arm Waypoints Created."

                self.current_state += 1

            elif self.current_state == 2:
                '''
                The goals of this state are:
                    1. Move to pick up the first set of blocks with first end effector.
                    2. Do this again for the second end effector and the second set of blocks.
                '''
                print "=== STATE 2 ==="
                print "> Executing waypoints."

                # The arm waypoints list has 2 elements composed of each of the end effectors waypoints.
                for ee,arm_waypoint in enumerate(arm_waypoints):
                    this_ee = [_ee for _ee in self.ee_list if _ee.frame_id == ("EE"+str(ee+1))]
                    
                    for gripper, block_position, grippers_to_acutate in arm_waypoint:
                        waypoint = self.waypoint_generator.get_block_waypoint(ee_number, gripper_number, block_number)

                        # Set nav waypoint then move the elevator to the correct height.
                        self.ros_manager.set_nav_waypoint(waypoint)
                        self.ros_manager.set_elevator(self.get_height_of_blocks())

                        estimated_distance_to_wall = self.ros_manager.block_wall_y - self.ros_manager.pose[1]
                        print "Estimated Distance to Wall:",estimated_distance_to_wall

                        # Extend the rail, grab the blocks, then pull it back in 
                        self.ros_manager.set_rail(get_distance_to_gripping_position(estimated_distance_to_wall, this_ee))
                        self.servo_controller.close_grippers(grippers_to_acutate)
                        self.ros_manager.set_rail(get_distance_to_gripping_position(0, this_ee))

                    # Go back to a point and then rotate for the other waypoint
                    self.ros_manager.set_nav_waypoint(self.ros_manager.pose - np.array([0,.2,0]), pos_acc = .1, rot_acc = .1, vel_profile=1)
                    self.ros_manager.set_nav_waypoint(self.ros_manager.pose - np.array([0,0,3.14]), pos_acc = .1, rot_acc = .1, vel_profile=1)

            elif self.current_state == 3:
                '''
                The goals of this state are:
                    1. Move to train box_4.
                    2. Release that color on the first end effector.
                    3. Move to next box with a color that we have. Repeat until at the last box.
                    4. Turn around and repeat with the other end effector - starting from box 1.
                    5. If there are still blocks we haven't gotten, move back to reprocess B blocks.
                '''
                print "=== STATE 3 ==="
                # We are going to iterate over 2 directions and to the 4 boxes. The first direction drops off ee1 and the second drops off ee2.
                # Right now we have to stop at each box, it would be good if we didnt have to.
                print "Blocks:",self.ee1
                print "Blocks:",self.ee2
                for direction,ee in enumerate(self.ee_list):
                    # First make sure the gripper has blocks in it.
                    if ee.holding == 0:
                        continue
                    # When direction is 0, we will be pointing one way, when it is 1, we will rotate to face the other way.
                    rotational_offset = direction*np.pi
                    print rotational_offset
                    box_number = 1
                    while box_number <= 4:
                        # Run through before we move and make sure the next box is the color of a block we are holding
                        holding_color = False
                        while not holding_color:
                            if direction == 0:
                                box = 'box_'+str(5-box_number)
                            else:
                                box = 'box_'+str(box_number)

                            grippers_to_drop = ee.which_gripper(train_box_colors[box])
                            if len(grippers_to_drop) is not 0:
                                # If there are blocks that match the color of the box, we can break from this loop and go to that box.
                                holding_color = True
                                break
                            print "No %s blocks detected - skipping that box."%(train_box_colors[box])
                            box_number += 1

                        print "This color:",train_box_colors[box],box
                        # Where should we go and what direction should we point
                        curr_waypoint = np.copy(self.waypoints[box])
                        curr_waypoint[2] += rotational_offset

                        # Move and then when we get there drop off the associated color blocks
                        self.ros_manager.set_nav_waypoint(curr_waypoint)
                        
                        print "Dropping blocks grippers:",grippers_to_drop
                        ee.drop(grippers_to_drop)
                        time.sleep(1)
                        if ee.holding == 0:
                            print "Empty gripper!"
                            break
                        box_number += 1

                if self.waypoint_generator.picked_up > 16:
                    self.waypoint_generator.picked_up = 0
                    break
                else:
                    # Go back and reprocess B blocks
                    self.current_state = 2

            else:
                break

            #if self.current_state > 50: self.current_state = 0
            rate.sleep()

class RosManager():
    def __init__(self,s):
        self.state_machine = s
        self.map_version_set = False
        # Initally zero, updated by particle filter later
        self.pose = np.array([0,0,0])

        # ROS inits
        self.nav_waypoint_pub = rospy.Publisher("/robot/nav_waypoint", PoseStamped, queue_size=1)
        self.state_pub = rospy.Publisher("/robot/current_state", Int8, queue_size=1)
        self.nav_start_pub = rospy.Publisher("/robot/start_navigation", StartNavigation, queue_size=1)
        self.ultrasonic_side_pub = rospy.Publisher("/robot/navigation/set_ultrasonic_side", String, queue_size=1)
        self.rail = rospy.Publisher("/robot/arms/rail", Float64, queue_size=2)


        request_map = rospy.Service('/robot/request_map', RequestMap, self.get_map)
        
        self.set_init_pose = rospy.ServiceProxy('/set_pose', SetPose)
        self.nav_waypoint = rospy.ServiceProxy('/robot/nav_waypoint', NavWaypoint)
        self.arm_waypoint = rospy.ServiceProxy('/robot/arm_waypoint', ArmWaypoint)
        self.set_elevator = rospy.ServiceProxy('/robot/arm/elevator_target', DynamixelControl)
        self.set_rail = rospy.ServiceProxy('/robot/arm/rail_target', DynamixelControl)

        rospy.Subscriber("/odometry/filtered", Odometry, self.got_odom, queue_size=1)

        rospy.init_node('main_control')
        
        rospy.Subscriber('/settings/start_command', Bool, self.recieve_start_command)
        rospy.Subscriber('/settings/map_version', Int8, self.determine_map_version)

    def got_odom(self,msg):
        msg = msg.pose
        yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])[2]
        self.pose = np.array([msg.pose.position.x,msg.pose.position.y,yaw])

    def recieve_start_command(self,msg):
        if msg.data: #and not self.state_machine.running:
            if self.state_machine.map_version:
                print "> State Machine Starting..."
                nav_start = StartNavigation()
                nav_start.map = self.get_map(None)[3]

                # 1 is the map configuration where we start on the left, 2 is on the right.
                    #self.state_machine.begin_1()
                if self.state_machine.map_version == 1:
                    start_pose = np.array([.2,2,3.1415])

                    self.start_ekf(start_pose)
                    nav_start.init_pose = start_pose
                    self.nav_start_pub.publish(nav_start)
                    self.state_machine.begin()

                elif self.state_machine.map_version == 2:
                    start_pose = np.array([self.far_wall_x - .2,.2,1.57])
                    nav_start.init_pose = start_pose
                    self.nav_start_pub.publish(nav_start)
            else:
                print "Error, no map version set."

    def start_ekf(self, position):
        q = tf.transformations.quaternion_from_euler(0, 0, position[2])
        header = Header(
            stamp=rospy.Time.now(),
            frame_id="map"
        )
        pose = Pose(
            position=Point(
                x=position[0],
                y=position[1],
                z=0
            ),
            orientation=Quaternion(
                x=q[0],
                y=q[1],
                z=q[2],
                w=q[3],
            )
        )
        # Publish pose with covariance stamped.
        p_c_s = PoseWithCovarianceStamped()
        p_c = PoseWithCovariance()
        # These don't matter
        covariance = np.array([.01,   0,  0,   0,   0,   0,
                                 0, .01,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   .0001])**2
        p_c.pose = pose
        p_c.covariance = covariance
        p_c_s.header = header
        p_c_s.header.frame_id = "map"
        p_c_s.pose = p_c

        self.set_init_pose(p_c_s)

    def determine_map_version(self,msg):
        self.state_machine.map_version = msg.data
        self.map_version_set = True

        print "> Map Version:",self.state_machine.map_version

    def set_nav_waypoint(self, waypoint, pos_acc = 0, rot_acc = 0, vel_profile = 2):
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
        print "> Nav Waypoint published:",waypoint
        print "> Moving..."
        success = self.nav_waypoint(p_s, pos_acc, rot_acc, vel_profile)
        print "> Movement Complete!"

    def set_elevator_height(self,height):
        self.elevator = rospy.Publisher("/robot/arms/elevator", Float64, queue_size=2)

    def set_arm_waypoint(self,gripper,waypoint):
        q = tf.transformations.quaternion_from_euler(0, 0, 1.5707)
        p_s = PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                ),
                pose=Pose(
                    position=Point(
                        x=waypoint[0],
                        y=waypoint[1],
                        z=waypoint[2]
                    ),
                    orientation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3],
                    )
                )
            )
        print "> Arm Waypoint published:",waypoint
        print "> Moving..."
        success = self.arm_waypoint(gripper,p_s)
        print "> Movement Complete!"

    def publish_current_state(self,state):
        self.state_pub.publish(state)

    def get_map(self,srv):
        self.block_wall_y = 2.174 #m
        self.far_wall_x = 2.438 #m
        map_1 = np.array([   0, 0, 0, 2.174,             1, # Left Wall
                             0, 0, 2.438, 0,             1, # Back Wall
                             2.438, 0, 2.438, 2.174,     1, # Right Wall
                             0, 2.174, 2.438, 2.174,     0, # Front Wall
                             # Tunnel
                             0, .76, .017, .76,          0, # Rear wall of left side of tunnel 
                             .017, .76, .017, 1.14,      0, # Inside face of left side of tunnel
                             0, 1.14, .017, 1.14,        0, # Front wall of left side of tunnel
                             0, 1.14, 0, 2.17,           0, # Left map wall up to Block area 
                             .440, .76, .456, .76,       0, # Rear wall of right side of tunnel
                             .456, .76, .456, 1.14,      0, # Ouside face of right side of tunnel
                             .440, .76, .440, 1.14,      0, # Inside face of right side of tunnel
                             .440, 1.14, .456, 1.14,     0, # Front wall of right side of tunnel
                             # Truck
                             .508, 0, .508, .303,        1, # Outside left side of truck wall
                             .520, .303, .528, .303,     1, # Front side of left truck wall
                             .528, .303, .528, 0,        1, # Inside left truck wall
                             .528, .07, .710, .07,       1, # Back wall of truck
                             .710, 0, .710, .303,        1, # Inside right truck wall
                             .710, .303, .730, .303,     1, # Fron side of right truck wall 
                             .730, .303, .730, 0,        1, # Outside right side of truck
                             # Train
                             2.158, .88, 2.438, .88,     0, # Back side of box_4
                             2.158, .88, 2.158, 1.16,    0, # Left side of box_4
                             2.158, 1.16, 2.438, 1.16,   0, # Front side of box_4
                             2.158, 1.185, 2.438, 1.185, 0, # Back side of box_3
                             2.158, 1.185, 2.158, 1.465, 0, # Left side of box_3
                             2.158, 1.465, 2.438, 1.465, 0, # Front side of box_3
                             2.158, 1.49, 2.438, 1.49,   0, # Back side of box_2
                             2.158, 1.49, 2.158, 1.77,   0, # Left side of box_2
                             2.158, 1.77, 2.438, 1.77,   0, # Front side of box_2
                             2.158, 1.795, 2.438, 1.795, 0, # Back side of box_1
                             2.158, 1.795, 2.158, 2.075, 0, # Left side of box_1
                             2.158, 2.075, 2.438, 2.075, 0, # Front side of box_1
                        ]).astype(np.float32) 

        # Flip map_2 over the y axis to get map_1
        map_2 = np.copy(map_1.reshape(len(map_1)/5,5)).T
        map_2[0] = self.far_wall_x - map_2[0]
        map_2[2] = self.far_wall_x - map_2[2]
        map_2 = map_2.T.flatten()

        if self.state_machine.map_version == 1:
            return self.state_machine.map_version, self.block_wall_y, self.far_wall_x, map_1
        elif self.state_machine.map_version == 2:
            return self.state_machine.map_version, self.block_wall_y, self.far_wall_x, map_2

if __name__ == "__main__":
    os.system("clear")
    #ShiaStateMachine()
    TestingStateMachine()
