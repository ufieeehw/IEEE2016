#!/usr/bin/env python
import time
## Math
import numpy as np
import math
import scipy.stats
## Ros/tools
import rospy
import tf
## Ros msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point32, PointStamped, Point, Quaternion, PoseWithCovarianceStamped, PoseWithCovariance
from sensor_msgs.msg import PointCloud, LaserScan
from ieee2016_msgs.msg import UltraSonicStamped, UltraSonicActivator

from ieee2016_msgs.srv import RequestMap


def make_2D_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    mat = np.array([
        [c,     -s],
        [s,      c],
    ],
    dtype=np.float32)
    return mat

class LidarPositionEstimator():
    '''
    With one of the lidars, estimate the position and rotation of the robot
    '''
    def __init__(self, wall_y = 2.172):
        self.tf_listener = tf.TransformListener()
        self.wall_y = wall_y
        self.pose_est_pub = rospy.Publisher("/robot/navigation/us_pose_vis", PoseStamped, queue_size=2) 
        self.p_c_s_est_pub = rospy.Publisher('/robot/navigation/us_pose', PoseWithCovarianceStamped, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan_trimmed', LaserScan, queue_size=10)
        rospy.Subscriber("/robot/navigation/lidar/scan_right", LaserScan, self.got_scan, queue_size=10)
        
        self.map_y = None

        self.FOV = 40

    def got_scan(self,msg):
        # Take all three scans, trim to a new FOV, convert them to cartesian, apply transformations, pub pointcloud, tranform back to cart, and pub laserscan
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)
        ranges_size = ranges.size
        self.frame_id = msg.header.frame_id

        # Trim to FOV
        new_FOV = math.radians(self.FOV)
        angle_min = -(new_FOV/2.0)
        angle_max = (new_FOV/2.0)
        ranges = ranges[int(ranges_size/2 + angle_min/angle_increment): int(ranges_size/2 + angle_max/angle_increment)]
        ranges_size = ranges.size

        msg.angle_min=angle_min
        msg.angle_max=angle_max
        msg.ranges = ranges
        self.scan_pub.publish(msg)
        # Get TF data about the LIDAR position
        trans = self.get_tf_link(time=rospy.Time(0))

        # Generate list that maps indices of the ranges to radian angle measures
        thetas = np.linspace(angle_min, angle_max, ranges_size, endpoint=True) + trans[2]
        cartesian_scan = np.apply_along_axis(self.polar_to_cart, 0, np.vstack((ranges,thetas)), trans[:2]).T# + np.array(trans[:2])

        cartesian_scan = cartesian_scan[~np.isnan(cartesian_scan)]
        cartesian_scan = cartesian_scan[~np.isinf(cartesian_scan)].reshape((-1,2))
        #sprint cartesian_scan
        # Draw a line through the PointStamped
        slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(cartesian_scan)
        self.std_err = std_err * 50
        print self.std_err
        # Each group has different parameters that should requires the pose estimate to be different.
        # Invert yaw here and change map_y
        # if self.activated_group == "left":
        map_y = self.wall_y + intercept
        map_yaw = np.arctan(-slope)+3.1416

        if not self.map_y:
            self.map_y = map_y
            self.map_yaw = map_yaw

        # elif self.activated_group == "right":
        #     map_y = -self.wall_y - intercept
        #     map_yaw = np.arctan(slope)
        
        # New data has these weights:
        yaw_factor = .2
        position_factor = .5
        self.map_y = (1-position_factor)*self.map_y + (position_factor)*map_y
        self.map_yaw = (1-yaw_factor)*self.map_yaw + (yaw_factor)*map_yaw

        print np.degrees(self.map_yaw), self.map_y
        self.publish_pose(np.array([0,self.map_y,self.map_yaw]),rospy.Time.now())

    def get_tf_link(self, target_frame="base_link", time=None):
        # Returns the tf link between this sensor and the target frame as close as we can to 'time'
        if time is None: time = self.tf_listener.getLatestCommonTime(target_frame, self.frame_id)
        self.tf_listener.waitForTransform(target_frame, self.frame_id, time, rospy.Duration(1.0))
        pos, quaternion = self.tf_listener.lookupTransform(target_frame, self.frame_id, time)
        rot = tf.transformations.euler_from_quaternion(quaternion)

        return np.array([pos[0], pos[1], rot[2]])

    def publish_pose(self,pose,time):
        #Generate pose
        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        header = Header(
            stamp=rospy.Time.now(),
            frame_id="map"
        )
        pose = Pose(
            position=Point(
                x=pose[0],
                y=pose[1],
                z=0
            ),
            orientation=Quaternion(
                x=q[0],
                y=q[1],
                z=q[2],
                w=q[3],
            )
        )

        # Publish pose stamped
        self.pose_est_pub.publish(
            PoseStamped(
                header=header,
                pose=pose
            )
        )

        # Publish pose with covariance stamped.
        p_c_s = PoseWithCovarianceStamped()
        p_c = PoseWithCovariance()
        # These don't matter
        covariance = np.array([  1,   0,  0,   0,   0,   0,
                                 0,.01*self.std_err,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,   0,
                                 0,   0,  0,   0,   0,  self.std_err])**2
        p_c.pose = pose
        p_c.covariance = covariance
        p_c_s.header = header
        p_c_s.header.frame_id = "map"
        p_c_s.pose = p_c
        # if time.time() - self.start_time < 5:
        #     self.p_c_s_init_pub.publish(p_c_s)
        # else:
        self.p_c_s_est_pub.publish(p_c_s)

    def polar_to_cart(self,r_theta,translation,frame=None):
        # Convert (r,theta) into (x,y)
        # Since the LIDAR are flipped and rotated in different directions, you have to change the signs of x and y based on the scan
        if frame == "laser_front" or frame == "laser_back":
            x = (r_theta[0] * np.cos(r_theta[1]) + translation[0])
            y = -(r_theta[0] * np.sin(r_theta[1]) + translation[1])
        else:
            x = -(r_theta[0] * np.cos(r_theta[1])) + translation[0]
            y = (r_theta[0] * np.sin(r_theta[1])) + translation[1]

        return x,y


class UltraSonicSensor():
    '''
    Defines an ultrasonic sensor.
    This will handle things like transforming ranges into base_link amoung other things.

    All measurements should be in meters.
    '''
    def __init__(self, frame_id, min_range, max_range, group, tf_listener):
        self.frame_id = frame_id
        self.tf_listener = tf_listener
        
        self.last_update = None
        self.range = 0

        self.min_range = min_range
        self.max_range = max_range
        self.group = group
    
    def get_tf_link(self, target_frame="base_link", time=None):
        # Returns the tf link between this sensor and the target frame as close as we can to 'time'
        if time is None: time = self.tf_listener.getLatestCommonTime(target_frame, self.frame_id)
        self.tf_listener.waitForTransform(target_frame, self.frame_id, time, rospy.Duration(1.0))
        pos, quaternion = self.tf_listener.lookupTransform(target_frame, self.frame_id, time)
        rot = tf.transformations.euler_from_quaternion(quaternion)

        return np.array([pos[0], pos[1], rot[2]])

class USPoseEstimator():
    '''
    Probably wont use due to not having Ultrasonic sensors.

    Given some number of ultrasonic sensors, generate a pose estimation. The estimation will only be accurate
    for the y position and the yaw.
    '''
    def __init__(self, wall_y, *sensors):
        self.sensors = sensors
        self.wall_y = wall_y
        self.point_cloud_pub = rospy.Publisher("/ultrasonic_points", PointCloud, queue_size=2) 
        self.pose_est_pub = rospy.Publisher("/robot/navigation/us_pose_est_vis", PoseStamped, queue_size=2) 
        self.p_c_s_est_pub = rospy.Publisher('/robot/navigation/us_pose_est', PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber("/robot/navigation/ultra_sonic/sensor_data", UltraSonicStamped, self.got_range, queue_size=10)
        rospy.Subscriber("/robot/navigation/ultra_sonic/activator", UltraSonicActivator, self.got_activator, queue_size=10)

        self.activated_group = None

        r = rospy.Rate(25) #hz
        while not rospy.is_shutdown():
            r.sleep()
            if self.activated_group is not None:
                self.estimate()

    def estimate(self):
        x = np.array([])
        y = np.array([])
        activated_sensors = np.copy(self.activated_sensors)
        for sensor in activated_sensors:
            # Put a check in here to make sure the sensor isn't out of range.
            position = sensor.get_tf_link(time=sensor.last_update)
            rot_mat = make_2D_rotation(position[2])
            position[:2] += np.dot(rot_mat,np.array([sensor.range,0]))

            x = np.append(x,position[0])
            y = np.append(y,position[1])

        self.publish_points(x,y)

        # Estimate line between points
        slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(x,y)

        stamp = activated_sensors[0].last_update
        # Each group has different parameters that should requires the pose estimate to be different.
        # Invert yaw here and change map_y
        if self.activated_group == "left":
            map_y = self.wall_y - intercept
            map_yaw = np.arctan(slope)
        elif self.activated_group == "right":
            map_y = -self.wall_y - intercept
            map_yaw = np.arctan(slope)

        print np.degrees(map_yaw), slope
        self.publish_pose_est(np.array([0,map_y,map_yaw]),stamp)

    def got_activator(self,msg):
        # Determine which group to activate
        if msg.activate:
            self.activated_group = msg.group_name
            # Get a list of activated sensors
            self.activated_sensors = []
            for sensor in self.sensors:
                if sensor.group == self.activated_group:
                    self.activated_sensors.append(sensor)
        else:
            self.activated_group = None


    def got_range(self,msg):
        frame_id = msg.header.frame_id
        # Go through and find which sensor we have recieved
        for index, sensor in enumerate(self.sensors):
            if sensor.frame_id == frame_id:
                this_sensor = self.sensors[index]
                this_sensor.last_update = msg.header.stamp
                break
        else:
            print "Sensor not found!"
            return

        this_sensor.range = msg.range
        #print this_sensor.frame_id,this_sensor.range

    def publish_pose(self,pose,stamp):
        #Generate pose
        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        header = Header(
            stamp=stamp,
            frame_id="map"
        )
        pose = Pose(
            position=Point(x=pose[0], y=pose[1], z=0),
            orientation=Quaternion( x=q[0], y=q[1], z=q[2], w=q[3])
        )

        # Publish pose stamped
        self.pose_est_pub.publish(
            PoseStamped(
                header=header,
                pose=pose
            )
        )

        self.br.sendTransform((self.pose_est[0], self.pose_est[1], .125), q,
                 rospy.Time.now(),
                 "base_link",
                 "map")

        # Publish pose with covariance stamped.
        p_c_s = PoseWithCovarianceStamped()
        p_c = PoseWithCovariance()
        covariance = np.array([.05,   0,   0,   0,   0,   0,
                                 0, .05,   0,   0,   0,   0,
                                 0,   0, .05,   0,   0,   0,
                                 0,   0,   0, .05,   0,   0,
                                 0,   0,   0,   0, .05,   0,
                                 0,   0,   0,   0,   0, .05])**2
        p_c.pose = pose
        p_c.covariance = covariance
        p_c_s.header = header
        p_c_s.pose = p_c
        self.p_c_s_est_pub.publish(p_c_s)

    def publish_pose_est(self,pose,stamp):
        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        header = Header(
            stamp=stamp,
            frame_id="map"
        )
        pose = Pose(
            position=Point(x=pose[0], y=pose[1], z=0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        self.pose_est.publish(
            PoseStamped(
                header=header,
                pose=pose
            )
        )

    def publish_points(self,x,y):
        points = []
        for i in range(len(x)):
            points.append(Point32(
                    x=x[i],
                    y=y[i],
                    z=0
                )
            )
        self.point_cloud_pub.publish(PointCloud(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                    ),
                points=points
            )
        )

if __name__ == "__main__":
    rospy.init_node("ultrasonic_listener")

    l = LidarPositionEstimator()

    rospy.spin()
