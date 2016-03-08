#!/usr/bin/env python
import rospy
import numpy as np
import tf
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Pose, PoseStamped, Quaternion, Point

from ieee2016_msgs.srv import ArmWaypoint, NavWaypoint

import time

"""
The goal here is to take some desired position and move the arm into that position.
This will require modifications to work with actual servos and stepper motors 
(getting correct position data and such).

We only have the Z and Y axies controllable (since the arms can move out to the side). 
So any X or yaw movement will be sent out as a waypoint to the robot.

For some terminology I use here:
    Arm : the whole assembly (elevator and linear rail).
    Gripper : the device used to pick up 1 single block.
    End Effector (EE) : the grouping of 4 grippers mounted to each end of the linear rail.
"""

def xyzw_array(quaternion):
    '''Convert quaternion to non-shit array #ThanksJake'''
    xyzw_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return(xyzw_array)

def make_2D_rotation_mat(angle):
    c, s = np.cos(angle), np.sin(angle)
    mat = np.array([
        [c,     -s],
        [s,      c],
    ],
    dtype=np.float32)
    return mat

class ArmController():
    def __init__(self):
        self.arm_waypoint = rospy.Publisher('/robot/arm_waypoint', PoseStamped, queue_size=2)
        self.nav_goal_pub = rospy.Publisher("/robot/nav_waypoint", PoseStamped, queue_size=2) #Just for visualization
        self.elevator = rospy.Publisher("/robot/arms/elevator", Float64, queue_size=2)
        self.rail = rospy.Publisher("/robot/arms/rail", Float64, queue_size=2)

        rospy.init_node('arm_controller')
        
        # For this we will use a service. When the arm moves to the desired location,
        # the service will return True to the person who called it
        rospy.Service('/robot/arm_waypoint', ArmWaypoint, self.move_arm)

        # For actually moving to the desired location.
        self.goto_nav_goal = rospy.ServiceProxy('/robot/nav_waypoint', NavWaypoint)
        
        self.tf_broad = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        rospy.spin()

    def move_arm(self,srv):
        # Find the position of the Gripper in the base_link frame
        t = self.tf_listener.getLatestCommonTime(srv.gripper, "/base_link")
        gripper_position, quaternion = self.tf_listener.lookupTransform(srv.gripper, "/base_link", t)
        gripper_rotation = tf.transformations.euler_from_quaternion(quaternion)

        # Find the waypoint in the map frame
        des_pose = self.tf_listener.transformPose("/map",srv.des_pose)
        self.arm_waypoint.publish(des_pose)

        des_yaw = tf.transformations.euler_from_quaternion(xyzw_array(des_pose.pose.orientation))[2] + gripper_rotation[2]
        # Desired position consists of the waypoint position with a corrective factor for the position of the gripper
        # with respect to the base_link.
        des_position = np.array([des_pose.pose.position.x, des_pose.pose.position.y]) + \
                       np.dot(make_2D_rotation_mat(des_yaw-gripper_rotation[2]),gripper_position[:2])

        # Line Shia up to the point and set the elevator height to the correct height
        self.nav_goal = np.array([des_position[0],des_position[1],des_yaw])
        self.publish_nav_goal("/map")
        self.set_elevator(des_pose.pose.position.z, frame="/map")

        return True

    def publish_nav_goal(self,frame_id):
        q = tf.transformations.quaternion_from_euler(0, 0, self.nav_goal[2])
        p_s = PoseStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=frame_id
                    ),
                pose=Pose(
                        position=Point(
                                x=self.nav_goal[0],
                                y=self.nav_goal[1],
                                z=.129
                            ),
                        orientation=Quaternion(
                                x=q[0],
                                y=q[1],
                                z=q[2],
                                w=q[3]
                            )
                    )
            )
        self.nav_goal_pub.publish(p_s)
        self.goto_nav_goal(p_s)

    def set_elevator(self, des_height, frame="base_link"):
        '''
        Set the elevator to some height in the tf frame: 'frame'.

        **These parameters needs to be set from the mech team.**
        
        '''
        # Convert frame to base_link if it isn't already
        t = self.tf_listener.getLatestCommonTime(frame, '/base_link')
        trans,rot = self.tf_listener.lookupTransform(frame, '/base_link', t)
        des_height -= trans[2]

        min_height = 0 #m
        max_height = .3 #m
        print "Sending",des_height
        if min_height <= des_height <= max_height:
            self.elevator.publish(Float64(data=des_height))
        else:
            print "Too high! Can not move to height:",des_height

a = ArmController()