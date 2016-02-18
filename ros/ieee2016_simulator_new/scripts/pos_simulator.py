#!/usr/bin/env python
import rospy
import numpy as np
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Pose, PoseStamped, Quaternion, Point

class Simulator():
    def __init__(self, starting_point):
        # ROS inits
        self.pose_est_pub = rospy.Publisher('/robot/pf_pose_est', PoseStamped, queue_size=10)
        rospy.init_node('simulator')
        rospy.Subscriber("/robot/twist", TwistStamped, self.got_twist)
        
        self.tf_broad = tf.TransformBroadcaster()

        self.pose = np.array(starting_point).astype(np.float32)

        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            self.publish_pose()

            rate.sleep()

    
    def control(self):
        self.publish_pose()

    def got_twist(self, msg):
        vel = np.array([msg.twist.linear.x/10.0,msg.twist.linear.y/10.0])

        c, s = np.cos(-self.pose[2]), np.sin(-self.pose[2])
        mat = np.array([
            [c,     -s],
            [s,      c],
        ])
        vel = np.dot(vel,mat)
        print vel
        self.pose += np.array([vel[0],vel[1],msg.twist.angular.z/100.0])

    def publish_pose(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
        p_s = PoseStamped(
                header=Header(
                        stamp=rospy.Time.now(),
                        frame_id="map",
                    ),
                pose=Pose(
                        position=Point(
                                x=self.pose[0],
                                y=self.pose[1],
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
        self.pose_est_pub.publish(p_s)

        self.tf_broad.sendTransform((self.pose[0],self.pose[1],0), 
                tf.transformations.quaternion_from_euler(0,0,self.pose[2]),
                rospy.Time.now(), "base_link", "map")

if __name__ == "__main__":
    s = Simulator([.2,.2,1.57])