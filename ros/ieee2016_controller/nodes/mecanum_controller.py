#!/usr/bin/env python
from __future__ import division
## Math
import numpy as np
import math
## Ros/tools
import tf
from tf import transformations as tf_trans
import rospy
## Ros msgs
from std_msgs.msg import Header
from geometry_msgs.msg import (Pose, PoseStamped, TwistStamped, Pose2D, PoseWithCovariance, Point, Quaternion, Vector3,
    TwistWithCovariance, TwistWithCovarianceStamped, Twist)
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from ieee2016_msgs.msg import Mecanum
from ieee2016_msgs.srv import StopMecanum, StopMecanumResponse, ResetOdom, ResetOdomResponse
from ieee2016_xmega_connector_ported.srv import * #Echo, EchoRequest, EchoResponse, SetWheelSpeeds

import time

class Controller(object):
    def __init__(self):
        '''Initialize Controller Object'''
        rospy.init_node('mecanum_controller')
    
        wheel_diameter = 104e-3 # 104 mm
        self.wheel_radius = wheel_diameter / 2.0

        ang_scale = 5.172
        # Create the 4x4 mecanum transformation matrix
        self.mecanum_matrix = self.wheel_radius * np.matrix([
            [+1, +1, +1, +1],  # Unitless! Shooting for rad/s
            [+1, -1, -1, +1],  # Unitless! Shooting for rad/s
            [ang_scale * +1, ang_scale * +1, ang_scale * -1, ang_scale * -1],  # Unitless! Shooting for rad/s
        ], dtype=np.float32) / 4.0  # All of the rows are divided by 4

        self.pose = np.array([0, 0, .001])
        self.odom_pub = rospy.Publisher('/robot/navigation/odom', Odometry, queue_size=10)    
        self.odom_twist_pub = rospy.Publisher('/robot/navigation/odom_twist', TwistWithCovarianceStamped, queue_size=10)  

        rospy.loginfo("----------Attempting to find set_wheel_speeds service-------------")
        rospy.wait_for_service('/robot/xmega_connector/set_wheel_speeds')
        rospy.loginfo("----------Wheel speed service found--------------")
        self.wheel_speed_proxy = rospy.ServiceProxy('/robot/xmega_connector/set_wheel_speeds', SetWheelSpeeds)
        
        # Twist subscriber
        self.twist_sub = rospy.Subscriber('/robot/twist', TwistStamped, self.got_twist, queue_size=2)
    
        rospy.loginfo("----------Attempting to find odometry service-------------")
        rospy.wait_for_service('/robot/xmega_connector/get_odometry')
        rospy.loginfo("----------Odometry service found--------------")

        self.odometry_proxy = rospy.ServiceProxy('/robot/xmega_connector/get_odometry', GetOdometry)
        rospy.Service('mecanum/stop', StopMecanum, self.stop)
        stop_proxy = rospy.ServiceProxy('/mecanum/stop', StopMecanum)
        rospy.Service('reset_odom', ResetOdom, self.reset)
        
        np.set_printoptions(precision=4)
        np.set_printoptions(suppress=True)
        
        self.on = True
        self.get_odom()
  
    def stop(self, req):
        self.on = not req.stop
        if not self.on:
            rospy.logwarn("DISABLING MECANUM DRIVE")
            self.wheel_speed_proxy(0.0, 0.0, 0.0, 0.0)
        else:
            rospy.logwarn("ENABLING MECANUM DRIVE")
        return StopMecanumResponse()

    def got_joy(self, joy_msg):
        if joy_msg.buttons[0]:
            rospy.loginfo("STOP!")
            stop_proxy(True)
    
    def got_twist(self, twist_stamped_msg):
        if not self.on:
            return
        twist_msg = twist_stamped_msg.twist

        desired_action = np.array([
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.angular.z,
        ],
        dtype=np.float32)
        #rospy.loginfo(desired_action)
        self.send_mecanum(desired_action)

    def got_twist_spacenav(self, twist_msg):
        if not self.on:
            return

        if self.spacenav_twist == twist_msg:
            return
        # Adding a deadzone to the spacenav controller
        dead_zone = .1
        if abs(twist_msg.linear.x) < dead_zone: twist_msg.linear.x = 0
        if abs(twist_msg.linear.y) < dead_zone: twist_msg.linear.y = 0
        if abs(twist_msg.angular.z) < dead_zone: twist_msg.angular.z = 0
        desired_action = np.array([
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.angular.z,
        ],
        dtype=np.float32)
        #rospy.loginfo(desired_action)
        self.send_mecanum(desired_action)
        self.spacenav_twist = twist_msg


    def send_mecanum(self, desired_action):
        '''Convert a desired linear and angular velocity vector into a wheel speed solution

        Wheel Orientations:
        (+) Means positive spin pushes robot toward the NUC's "THIS WAY UP" sticker
        (-) Means it pushes in the opposite direction
            Wheel_4 (+)  ...........  Wheel_3 (-)
            Wheel_1 (+)  ...........  Wheel_2 (-)

        Equations from [1]
        V_y = (V_0 + V_1 + V_2 + V_3) / 4
        V_x = (V_0 - V_1 + V_2 - V_3) / 4
        V_theta  = (V_0 + V_1 - V_2 - V_3) / 4
        V_error = (V_0 - V_1 - V_2 + V_3) / 4

        Function:
            (If you don't understand this, I suggest the Khan Academy Linear Algebra series
                very short and sweet)
            Precompute the left-inverse of the mecanum characteristic matrix
            Multiply that by the desired velocity
        ****This is equivalent to the least-squares solution****
        Proof:  
            Motivation:
                We would like to find a vector x* such that |b - x*| is minimized
                 (In this case, b is the 'vector' of desired [xvel, yvel, angvel])
                The vector that minimizes |b - x*| is the projection of b onto the column
                space (Call this subspace V) of A.

            So...
                Ax* = b_proj
                (Ax* - b) is orthogonal to V therefore,
                (Ax* - b) is a member of the null-space of A.T (Also called the left null-space)

            Then by definition of null-space,
                A.T(x* - b) = <0>
                Where <0> is the zero vector

            Distribute:
                (A.T * Ax*) - (A.T * <b>) = <0>
                A.T * Ax* = A.T * <b>
                x* = (A.T * A).I ( A.T * <b>)  
                For those keeping track at home, (A.T * A).I * A.T is the Moore-Penrose Left Psuedo-inverse of A
            Q.E.D.


        Notes:
            This approach is a slight departure from Forrest and Khaled's method from last year

        Bibliography:
            [1] http://www2.informatik.uni-freiburg.de/~grisetti/teaching/ls-slam/lectures/pdf/ls-slam-03-hardware.pdf

        '''
        v_target = np.array([desired_action[0], desired_action[1], desired_action[2]])
        mecanum_speeds = np.linalg.lstsq(self.mecanum_matrix, v_target)[0]

        wheel_speeds = [
            -mecanum_speeds[0], 
            mecanum_speeds[1], 
            mecanum_speeds[2], 
            -mecanum_speeds[3] 
        ]
        self.wheel_speed_proxy(*wheel_speeds)


    def make_2D_rotation(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        mat = np.matrix([
            [c,     -s],
            [s,      c],
        ],
        dtype=np.float32)
        return mat

    def reset(self, srv):
        self.pose = np.array([srv.x, srv.y, srv.yaw])
        return ResetOdomResponse()

    def get_odom(self):
        '''get_odom: at a rate of _freq_, compute the motion of the vehicle from wheel odometry
        Each item (pose, odom) is formatted as [x, y, theta]
        Odometry messages are used because they contain covariances
        '''
        freq = 10 #hz
        r = rospy.Rate(freq)
        start = time.time()
        while not rospy.is_shutdown():
            odom_srv = self.odometry_proxy()
            wheel_odom = np.array([
                -odom_srv.wheel1,
                odom_srv.wheel2,
                odom_srv.wheel3,
                -odom_srv.wheel4,
            ])
            vehicle_twist = np.dot(self.mecanum_matrix, wheel_odom).A1*10

            rot_mat = self.make_2D_rotation(self.pose[2])
            x, y = np.dot(rot_mat, [vehicle_twist[0], vehicle_twist[1]]).A1

            self.pose += [x, y, vehicle_twist[2]]

            header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='base_link',
                )

            t_c =TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=vehicle_twist[0],
                        y=vehicle_twist[1],
                        z=0.0,
                    ),
                    angular=Vector3(
                        x=0.0,
                        y=0.0,
                        z=vehicle_twist[2],
                    )
                ),
                covariance=np.diag([0.03**2] * 6).flatten()
            )

            orientation = tf_trans.quaternion_from_euler(0, 0, self.pose[2])
            odom_msg = Odometry(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id='map',
                ),
                child_frame_id='base_link',
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=self.pose[0],
                            y=self.pose[1],
                            z=0.0,
                        ),
                        orientation=Quaternion(*orientation),
                    ),
                    covariance=np.diag([.03**2] * 6).flatten(), # No real covariance, just uncertainty
                ),
                twist=t_c
            )
            t_c_s = TwistWithCovarianceStamped(
                header=header,
                twist=t_c
            )
            self.odom_pub.publish(odom_msg)
            self.odom_twist_pub.publish(t_c_s)
            r.sleep()


if __name__=='__main__':
    controller = Controller()
    rospy.spin()
