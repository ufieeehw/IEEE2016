#!/usr/bin/env python
import rospy
import numpy as np
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Pose, PoseStamped, Quaternion, Point

from ieee2016_msgs.srv import ArmWaypoint

import time

"""
The goal here is to take some desired position and move the arm into that position.
This will require modifications to work with actual servos and stepper motors 
(getting correct position data and such).

We only have the Z and Y axies controllable (since the arms can move out to the side). 
So any X or yaw movement will be sent out as a waypoint message to the robot in the 
correct direction.

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
        # This needs to be to the serovs, for now just go to tf
        self.arm_waypoint = rospy.Publisher('/arm/waypoint', PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher("/robot/twist", TwistStamped, queue_size=2)
        self.nav_goal_pub = rospy.Publisher("/robot/waypoint", PoseStamped, queue_size=2)
        rospy.init_node('arm_controller')

        
        # For this we will use a service. When the arm moves to the desired location,
        # the service will return that to the person who called it
        rospy.Service('/robot/arms/right/set_waypoint', ArmWaypoint, self.move_arm)
        
        self.tf_broad = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        rospy.spin()

    def move_arm(self,srv):
        print srv.gripper
        # Find the position of the Gripper in the base_link frame
        t = self.tf_listener.getLatestCommonTime(srv.gripper, "/base_link")
        gripper_position, quaternion = self.tf_listener.lookupTransform(srv.gripper, "/base_link", t)
        gripper_rotation = tf.transformations.euler_from_quaternion(quaternion)

        # Find the waypoint in the map frame
        des_pose = self.tf_listener.transformPose("/map",srv.des_pose)
        self.pub_pose(des_pose)

        des_yaw = tf.transformations.euler_from_quaternion(xyzw_array(des_pose.pose.orientation))[2] + gripper_rotation[2]
        # Desired position consists of the waypoint position with a corrective factor for the position of the gripper
        # with respect to the base_link and a buffer distance of .2 m
        des_position = np.array([des_pose.pose.position.x, des_pose.pose.position.y]) + \
                       np.dot(make_2D_rotation_mat(des_yaw-gripper_rotation[2]),gripper_position[:2]) - \
                       np.dot(make_2D_rotation_mat(des_yaw-gripper_rotation[2]),np.array([.2,0]))


        # Line Shia up infront (.2m) of the point and then switch to LIDAR navigation to get right infront of the wall
        self.nav_goal = np.array([des_position[0],des_position[1],des_yaw])
        self.publish_nav_goal("/map")



        #time.sleep(1)
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
                                z=0
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
    def pub_pose(self,pose):
        #q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        # p_s = PoseStamped(
        #         header=Header(
        #                 stamp=rospy.Time.now(),
        #                 frame_id="map",
        #             ),
        #         pose=pose
        #     )
        self.arm_waypoint.publish(pose)
        
a = ArmController()