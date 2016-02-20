#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int8, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
from ieee2016_msgs.srv import NavWaypoint, ArmWaypoint
import tf

import math
import numpy as np

def make_2D_rotation_mat(angle):
    c, s = np.cos(angle), np.sin(angle)
    mat = np.array([
        [c,     -s],
        [s,      c],
    ],
    dtype=np.float32)
    return mat


class CloseApproach():
    def __init__(self):
        self.max_linear_speed = .25 #m/s
        self.desired_distance = .0245 #m

        self.twist_pub = rospy.Publisher("/robot/twist", TwistStamped, queue_size=5)
        rospy.Subscriber("/lidar/sim_scan_comb", LaserScan, self.got_laserscan)

        self.FOV = math.radians(20)
        self.on = False

    def approach(self, direction):
        while not self.on: print "Waiting for scan..."

        ranges = self.laserscan.ranges

        # Trim to FOV
        angle_min = -(self.FOV/2.0)
        angle_max = (self.FOV/2.0)
        mid_point = (direction - self.laserscan.angle_min)/self.laserscan.angle_increment
        ranges_new = ranges[int(mid_point + angle_min/self.laserscan.angle_increment): \
                            int(mid_point + angle_max/self.laserscan.angle_increment)]

        # Check if we are close enough
        dist = sum(ranges_new)/len(ranges_new)
        if dist <= self.desired_distance: 
            self.on = False
            print "Desired distance reached."
            return True

        print "Distance to wall:", dist

        err = dist - self.desired_distance
        linear_vel = np.array([min(math.sqrt(err),self.max_linear_speed),0])
        print make_2D_rotation_mat(-direction)
        vel = np.dot(linear_vel,make_2D_rotation_mat(-direction))

        print "Velocity:"
        print vel
        self.publish_twist(vel)
        print
    def got_laserscan(self,msg):
        self.on = True
        self.laserscan = msg

    def publish_twist(self, vel):
        t_s = TwistStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id="laser_left"
                ),
                twist=Twist(
                        linear=Vector3(
                                x=vel[0],
                                y=vel[1],
                                z=0
                            ),
                        angular=Vector3(
                                x=0,
                                y=0,
                                z=0
                            )
                    )

            )
        self.twist_pub.publish(t_s)

if __name__ == "__main__":
    rospy.init_node("close_approach")
    c = CloseApproach()

    rate = rospy.Rate(100) #hz
    while not rospy.is_shutdown():
        if c.approach(1.57):
            break
        rate.sleep()