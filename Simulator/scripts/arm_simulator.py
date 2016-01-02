#!/usr/bin/python
## Math
import numpy as np
## Display
import pygame
import time
import math
## Ros
import rospy
from tf import transformations as tf_trans
## Ros Msgs
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from dynamixel_msgs.msg import JointState
from sim_tools import Text_Box

'''This simiulator was adopted from the 2015 robot.
   It displays in radians the elbow and shoulder angles.
   The yellow circle is a visual aid for the workspace of the arm.
   The red line is the elbow link. The purple line is the shoulder link. 

   TODO:
   -Add Z direction movement
   -Add second arm
   -Indicate link interference
   -Add grippers and environment

'''

SCREEN_DIM = (1000,1000)#(750, 750)
ORIGIN = np.array([SCREEN_DIM[0]/2.0, SCREEN_DIM[1]/2.0])


def print_in(f):
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name + " with arguments " + str(args) + " and kwargs " + str(kwargs))
        result = f(*args, **kwargs)
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)


def round_point((x, y)):
    '''Round and change point to centered coordinate system'''
    return map(int, ((1000 * x) + ORIGIN[0], -(1000 * y) + ORIGIN[1]))


def unround_point((x, y)):
    '''Change center-origin coordinates to pygame coordinates'''
    return ((x - ORIGIN[0]) / 1000.0, (-y + ORIGIN[1]) / 1000.0)


def norm_angle_diff(ang_1, ang_2):
    '''norm_angle_diff(ang_1, ang_2)
    -> Normalized angle difference, constrained to [-pi, pi]'''
    return (ang_1 - ang_2 + np.pi) % (2 * np.pi) - np.pi


class SCARA(object):
    def __init__(self):
        '''This intends to simulate the physical servo angles without fake offsets'''
        rospy.init_node('SCARA_simulator')

        # Offsets (Calibration)
        # self.shoulder_angle_offset = 1.06
        # self.elbow_angle_offset = 0.65
        # self.base_angle_offset = 0.0
        self.shoulder_angle_offset = 1.06
        self.elbow_angle_offset = 0.65
        self.base_angle_offset = 0.0

        # Position initalization
        self.base = np.array([0.0, 0.0], np.float32)
        self.shoulder_length, self.elbow_length = 0.148, 0.160
        self.elbow_joint = np.array([self.shoulder_length, 0.0], np.float32)
        self.wrist_joint = self.elbow_joint + np.array([self.elbow_length, 0.0], np.float32)

        # Base
        self.point = np.array([0.0, 0.0], np.float32)
        self.starting = np.array([.308, 0.0], np.float32)

        # ROS elements
        self.elbow_sub = rospy.Subscriber('elbow_controller/command', Float64, self.got_elbow_angle, queue_size=1)
        self.shoulder_sub = rospy.Subscriber('shoulder_controller/command', Float64, self.got_shoulder_angle, queue_size=1)
        self.error_sub = rospy.Subscriber('arm_des_pose', PointStamped, self.got_des_pose)

        self.elbow_pub = rospy.Publisher('elbow_controller/state', JointState, queue_size=1)
        self.shoulder_pub = rospy.Publisher('shoulder_controller/state', JointState, queue_size=1)
        self.wrist_pub = rospy.Publisher('wrist_controller/state', JointState, queue_size=1)

        # Message defaults
        self.shoulder_angle, self.elbow_angle = 0.0, 0.0
        self.elbow_servo_angle, self.shoulder_servo_angle = 0.0, 0.0
        self.position = None

    def got_des_pose(self, msg):
        '''Recieved desired arm pose'''
        self.position = (msg.point.x, msg.point.y, msg.point.z)

    def got_elbow_angle(self, msg):
        '''Recieved current elbow angle'''
        self.elbow_servo_angle = msg.data
        self.elbow_angle = -(msg.data + self.elbow_angle_offset)

    def got_shoulder_angle(self, msg):
        '''Recieved current base angle'''
        self.shoulder_servo_angle = msg.data
        self.shoulder_angle = msg.data + self.shoulder_angle_offset

    def publish_joint_angles(self):
        self.elbow_pub.publish(JointState(current_pos=self.elbow_servo_angle))
        self.shoulder_pub.publish(JointState(current_pos=self.shoulder_servo_angle))
        self.wrist_pub.publish(JointState(current_pos=0.0))

    def update(self, center=np.array([0, 0], np.float32)):
        '''Update each arm joint position according to the angles and lengths'''
        # TODO:
        # Make this non-instantaneous
        
        _shoulder_angle = self.shoulder_angle 
        _elbow_angle = self.elbow_angle 

        self.base = center
        # Update endpoint of link from shoulder to elbow
        shoulder_local_pos = self.shoulder_length * np.array([
            np.cos(_shoulder_angle), 
            np.sin(_shoulder_angle)
        ])

        self.elbow_joint = shoulder_local_pos + self.base

        # Update endpoint as sum of base angle and elbow angle
        # total_elbow_angle = _shoulder_angle - _elbow_angle
        total_elbow_angle = _elbow_angle

        # Superimpose positions
        elbow_local_pos = self.elbow_length * np.array([np.cos(total_elbow_angle), np.sin(total_elbow_angle)])
        self.wrist_joint = self.elbow_joint + elbow_local_pos


    def draw(self, display, new_base=(0, 0)):
        '''Draw the whole arm'''
        # Update positions given current angles
        self.update()
        # Draw the links
        pygame.draw.line(display, (255, 255, 255), (20, 0), (20,250), 3)
        if (self.position != None):
            pygame.draw.line(display, (0, 0, 255), (0, (0.5-self.position[2])*500), (40,(0.5-self.position[2])*500), 3)
            Text_Box.draw(display, 
                pos=(20,260),
                color=(250, 20, 30), 
                text="Height(Z): {}\n".format(self.position[2]),
            )
        pygame.draw.line(display, (255, 0, 255), round_point(self.base), round_point(self.elbow_joint), 4)

        pygame.draw.line(display, (255, 0, 255), round_point(self.base), round_point(self.elbow_joint), 4)
        pygame.draw.line(display, (255, 0, 0), round_point(self.elbow_joint), round_point(self.wrist_joint), 4)
        # Draw the desired position circle
        pygame.draw.circle(display, (255, 255, 50), round_point(self.base), int((self.shoulder_length + self.elbow_length) * 1000), 1)

        Text_Box.draw(display, 
            pos=round_point(self.elbow_joint),
            color=(250, 20, 30), 
            text="Elbow Angle: {}\n".format(round(self.elbow_angle, 4)),
        )
        Text_Box.draw(display, 
            pos=round_point(self.base),
            color=(250, 20, 30), 
            text="Shoulder Angle: {}\n".format(round(self.shoulder_angle, 4)),
        )

        if self.position is not None:
            pygame.draw.circle(display, (250, 30, 30), round_point((self.position[0],self.position[1])), 5, 1)

        # Publish joinstates
        self.publish_joint_angles()

def main():
    '''In principle, we can support an arbitrary number of arms in simulation'''
    arms = [
        SCARA(),

    ]

    display = pygame.display.set_mode(SCREEN_DIM)
    pygame.display.set_caption("Arm Simulation")

    des_pose_pub = rospy.Publisher('arm_des_pose', PointStamped, queue_size=1)



    def publish_des_pos((x, y, z)):
        '''Publish desired position of the arm end-effector based on click position'''
        des_pose_pub.publish(
            PointStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/robot',
                ),
                point=Point(
                    x=x, 
                    y=y, 
                    z=z,
                )
            )
        )

    clock = pygame.time.Clock()

    target_point = np.array([0.0, 0.0, 0.0])
    extent = 0.0
    base_angle = 0.0
    z = 0.0
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

            if event.type == pygame.KEYDOWN:
                if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                    return

                if event.key == pygame.K_a:
                    pt = pygame.mouse.get_pos()
                    extent, z = unround_point(pt)
                    _x, _y = extent * np.cos(base_angle), extent * np.sin(base_angle)
                    print 'Targeting position ({}, {}, {})'.format(_x, _y, z)
                    publish_des_pos((_x, _y, z))


                if event.key == pygame.K_b:
                    pt = pygame.mouse.get_pos()
                    x, y = unround_point(pt)
                    base_angle = np.arctan2(y, x)
                    _x, _y = extent * np.cos(base_angle), extent * np.sin(base_angle)
                    print 'Targeting position ({}, {}, {})'.format(_x, _y, z)
                    publish_des_pos((_x, _y, z))

        t = time.time()
        for arm in arms:
            arm.draw(display)
        
        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))


if __name__ == '__main__':
    main()
