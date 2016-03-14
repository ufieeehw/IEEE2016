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
from sensor_msgs.msg import PointCloud
from ieee2016_msgs.msg import UltraSonicStamped, UltraSonicActivator


def make_2D_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    mat = np.array([
        [c,     -s],
        [s,      c],
    ],
    dtype=np.float32)
    return mat

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
        p_c_s.header
        p_c_s.pose = p_c
        self.p_c_s_est_pub.publish(p_c_s)

    def publish_pose_est(self,pose,stamp):
        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        header = Header(
            stamp=stamp,
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
    tf_listener = tf.TransformListener()
    min_range = .01
    max_range = 1
    us1 = UltraSonicSensor("us_1", min_range, max_range, "left", tf_listener)
    us2 = UltraSonicSensor("us_2", min_range, max_range, "left", tf_listener)
    us3 = UltraSonicSensor("us_3", min_range, max_range, "right", tf_listener)
    us4 = UltraSonicSensor("us_4", min_range, max_range, "right", tf_listener)
    time.sleep(1)
    p = USPoseEstimator(1, us1,us2,us3,us4)
    rospy.spin()
