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
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3
from ieee2015_msgs.msg import Mecanum
from ieee2015_msgs.srv import StopController, StopControllerResponse
from nav_msgs.msg import Odometry

# max_linear_vel = 1 # m/s
max_linear_vel = 0.09
max_linear_acc = max_linear_vel # m/s^2

# max_angular_vel = 2 # rad/s
max_angular_vel = 0.5 # rad/s
max_angular_acc = max_angular_vel # rad/s^2 

# (Jason says this is just called angular acceleration, # I call it angcelleration)


def print_in(f):
    '''Shitty decorator for printing function business'''
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name)
        result = f(*args, **kwargs)
    
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)


def xyzw_array(quaternion):
    '''Convert quaternion to non-shit array'''
    xyzw_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return(xyzw_array)


class Controller(object):
    '''Controller object 
    See Jacob Panikulam or Aaron Marquez for questions

    Function:
        - Determine the ideal velocity for the current position

    Notes:
        Special glory to Lord Forrest Voight, creator of the universe
        Greater thanks to the incredible, brilliant and good-looking Khaled Hassan, for his unyielding mentorship
        and love for Burrito Bros
    '''
    def __init__(self):
        '''Initialize Controller Object'''
        rospy.init_node('vehicle_controller')

        # Twist pub
        self.twist_pub = rospy.Publisher('twist', TwistStamped, queue_size=1) 
        
        # Initializations to avoid weird desynchronizations
        self.des_position = None
        self.des_yaw = None
        # Don't want to have an a-priori position
        self.position = None
        self.yaw = None

        # Current pose sub
        self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.got_pose)
        #self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.got_odom)

        self.desired_pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.got_desired_pose)

        self.on = True
        rospy.Service('controller/stop', StopController, self.stop)
        freq = 10
        r = rospy.Rate(freq) # 10hz

        while not rospy.is_shutdown():
            rospy.sleep(rospy.Duration(0.1))
            self.control()
            r.sleep()


    def stop(self, req):
        self.on = not req.stop
        if not self.on:
            rospy.logwarn("DISABLING MECANUM CONTROLLER")
            self.des_position = None
            self.des_yaw = None
        else:
            rospy.logwarn("ENABLING MECANUM CONTROLLER")
            self.des_position = None
            self.des_yaw = None

        return StopControllerResponse()

    def send_twist(self, (xvel, yvel), angvel):
        '''Generate twist message'''
        self.twist_pub.publish(
            TwistStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='base_link',
                ),
                twist=Twist(
                    linear=Vector3(xvel, yvel, 0),
                    angular=Vector3(0, 0, angvel),  # Radians
                )
            )
        )

    def norm_angle_diff(self, ang_1, ang_2):
        '''norm_angle_diff(ang_1, ang_2)
        -> Normalized angle difference, constrained to range [-pi, pi]
        '''
        return ((ang_1 - ang_2 + np.pi) % (2 * np.pi)) - (np.pi)

    def unit_vec(self, v):
        '''unit_vec(v)'''
        norm = np.linalg.norm(v)
        if norm == 0:
            return(v)
        return np.array(v) / np.linalg.norm(v)

    def vec_diff(self, v1, v2):
        '''norm(v1 - v2)'''
        assert isinstance(v1, np.array)
        assert isinstance(v2, np.array)
        diff = np.linalg.norm(v1 - v2)
        return(diff)

    def sign(self, x):
        '''return sign of x -> -1, 1 or 0'''
        if x > 0: return 1
        elif x < 0: return -1
        else: return 0

    def got_odom(self, msg):

        pose = msg.pose.pose
        self.position = np.array([pose.position.x, pose.position.y])
        self.yaw = tf_trans.euler_from_quaternion(xyzw_array(pose.orientation))[2]


    def got_pose(self, msg):
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

    def control(self):
        '''recieve current pose of robot

        Function:
            Attempts to construct a velocity solution using a k*sqrt(error) controller
            This tries to guarantee a constant acceleration

        Note:
            Right now, this does not require velocity feedback, only pose
            This SHOULD include velocity feedback, once we have it

        Todo:
            Add speed-drop for the case where position feedback is lost
            This will probably have to be done in a separate thread

        Changes from last year:
            Enabled backwards and side to side motion.

        Velocity calcluation should be done in a separate thread
         this thread should have an independent information "watchdog" timing method
        '''
        if (self.des_position is None) or (self.des_yaw is None) or (self.on is False):
            #rospy.logwarn("des")
            return

        if (self.position is None) or (self.yaw is None):
            #rospy.logwarn("curr")
            return

        # World frame position
        position_error = self.des_position - self.position
        yaw_error = self.norm_angle_diff(self.des_yaw, self.yaw)

        position_error_len = np.linalg.norm(position_error)

        if (position_error_len > 0.08): # 8cm stop-error
            #yaw_target = np.arctan2(position_error[0], -position_error[1]) - (np.pi /2)
            #yaw_error = self.norm_angle_diff(yaw_target, self.yaw)

            # print 'error', position_error
            # print 'yaw error', yaw_error
            if np.fabs(yaw_error) > 0.08:
                state = 'nav_start_rotate'
            else:
                state = 'nav_drive'

        elif np.fabs(yaw_error) > 0.05:
            state = 'stop_rotate'

        else:
            state = 'stop'

        # Determines the linear speed necessary to maintain a consant backward acceleration
        linear_speed = min(
                            math.sqrt(2 * np.linalg.norm(position_error) * max_linear_acc),
                            max_linear_vel
                        )
        # Determines the angular speed necessary to maintain a constant angular acceleration 
        #  opposite the direction of motion
        angular_speed = min(
                            math.sqrt(2 * abs(yaw_error) * max_angular_acc), 
                            max_angular_vel
                        )

        # Provide direction for both linear and angular velocity
        desired_vel = linear_speed * self.unit_vec(position_error)
        print desired_vel
        desired_angvel = angular_speed * self.sign(yaw_error)

        # Unit vectors that determine the right-handed coordinate system of the roobt in the world frame
        #self.yaw = math.radians(90)
        #forward = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        #left = np.array([math.cos(self.yaw + np.pi/2), math.sin(self.yaw + np.pi/2)])
        rot_mat = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                            [math.sin(self.yaw),  math.cos(self.yaw)]])
        print rot_mat

        # Send twist if above error threshold
        # Point at position, go to position, achieve desired orientation

        print state
        if state == 'nav_drive':
            vel = rot_mat.dot(desired_vel)
            target_angvel = desired_angvel

        elif state == 'nav_start_rotate':
            vel = [0,0]
            target_angvel = desired_angvel

        elif state == 'stop_rotate':
            vel = [0,0]
            target_angvel = desired_angvel

        elif state == 'stop':
            vel = [0,0]
            target_angvel = 0.0

        self.send_twist(vel, target_angvel) 

    def got_desired_pose(self, msg):
        '''Recieved desired pose message
        Figure out how to do this in a separate thread
        So we're not depending on a message to act
        #LearnToThreading
        (That's a hashtag)
        '''
        self.des_position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.des_yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]


if __name__ == '__main__':
    rospy.logwarn("Starting")
    controller = Controller()
    rospy.spin()
    
