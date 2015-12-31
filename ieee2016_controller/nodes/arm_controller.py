#!/usr/bin/python
from __future__ import division
## Math
import numpy as np
## Ros
import rospy
import tf.transformations as tf_trans
import tf
## Ros Msgs
from std_msgs.msg import Header, Float32, Float64
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
#NOT HARDWARE READY
'''DO NOT RUN ON HARWARE. THIS CODE IS NOT CURRETLY SAFE AND WILL BREAK THINGS
   This "Controller" (Angle Solver) was adapted from the ieee_2015 robot. 
   Currently implemented is the left arm. The left shoulder link is restricted to the 
   FIRST and SECOND quadrants and angle between the elbow and shoulder links is    
   restricted to -pi/2<theta<pi/2 measured from the x unit vector on the shoulder
   link. If a desired point is given that would contradict these the arm moves to a   
   reflection of the point across the x-axis. This is not intentional.

   TODO:
   -Fix and Add constraints to match anticipated hardware
   -Add the second arm
   -Add authority in Z direction
   -Add a path planner
   - 
'''

class SCARA_Controller(object):
    '''APPLIES ONLY TO 2 DOF ARM, PROOF OF CONCEPT'''
    def __init__(self):
        rospy.init_node('SCARA_controller')
        self.shoulder_length, self.elbow_length = (0.148, 0.160)
        self.base = np.array([0.0, 0.0], np.float32)

        ## Offsets
        # Angles
        self.shoulder_angle_offset = 1.06
        self.elbow_angle_offset = 0.65

        # Distances
        self.base_arm_offset = 0.0  # in x direction

        self.pose_sub = rospy.Subscriber('arm_des_pose', PointStamped, self.got_des_pose, queue_size=1)

        self.elbow_pub = rospy.Publisher('elbow_controller/command', Float64, queue_size=1)
        self.shoulder_pub = rospy.Publisher('shoulder_controller/command', Float64, queue_size=1)

        self.des_position = None

    def got_des_pose(self, msg):
        '''Receiving desired pose'''
        self.des_position = np.array([msg.point.x, msg.point.y, msg.point.z])
        
        scara_x = msg.point.x
        scara_y = msg.point.y
        arm_solution = self.solve_arm_angles((scara_x, scara_y))

        if (arm_solution is not None):
            shoulder, elbow = arm_solution

            self.publish_angles(shoulder, elbow)
        else:
            rospy.logwarn("Could not find an arm movement solution")

    def publish_angles(self, shoulder, elbow):
        '''Publish shoulder and elbow angles
        '''

        # Apply the inverse of the angle correction to get the servo angles from arm angles
        _shoulder_angle = shoulder - self.shoulder_angle_offset
        _elbow_angle = np.pi -(elbow + self.elbow_angle_offset)


        self.shoulder_pub.publish(Float64(data=_shoulder_angle))
        self.elbow_pub.publish(Float64(data=_elbow_angle))


    def solve_arm_angles(self, pt):
        '''solve_arm_angles((x, y)) -> Elbow and shoulder angles
        This computes the angles given a point in the SCARA plane

        2DOF has a closed form solution, this method only slightly extends to higher DOF, where there is not a closed form

        Returns None if no solution exists.
        You should *NEVER* get the numpy "Invalid input in arccos" or a NaN output - if you do, something went wrong.'''
        x, y = pt
        #Constraints
        if(x==0.0 and y==0.0):return None

        
        distance = np.sqrt((x**2) + (y**2))

        # u-arm (Elbow-Angle wrt robot x unit vector)
        c2 = ((self.elbow_length**2 + self.shoulder_length**2) - (x**2 + y**2))/(2 * self.elbow_length * self.shoulder_length)
        s2 = np.sqrt(1 - c2**2)

        c1 = (self.shoulder_length * x - self.elbow_length * (x * c2 + y * s2))/(self.shoulder_length**2 + ((self.elbow_length * s2)**2 + (self.elbow_length * c2)**2) - 2 * self.shoulder_length * self.elbow_length * c2)
        shoulder_angle = np.arccos(c1)
        elbow_angle = shoulder_angle + np.arccos(c2)
        
        return (shoulder_angle, elbow_angle)



if __name__ == '__main__':
    SCARA = SCARA_Controller()
    rospy.spin()
