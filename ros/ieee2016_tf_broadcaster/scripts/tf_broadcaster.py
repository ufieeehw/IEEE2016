#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry


class TFPublisher():
    def __init__(self,rate):
        self.tf_broad = tf.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber("/robot/odom", Odometry, self.got_odom, queue_size=2)
        # base_link (0,0,0) = center base(top level) of Shia

        r = rospy.Rate(rate) #hz
        while not rospy.is_shutdown():
            print "Transforming..."
            # Publish all of our static transformations
            self.static_tf()

            r.sleep()

    def got_odom(self,msg):
        msg = msg.pose.pose
        self.tf_broad.sendTransform((msg.position.x,msg.position.y,0),(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),rospy.Time.now(),"odom_scan","map")

    def static_tf(self):
        # End Effector 1 -> Each Gripper, G0 is the far left
        self.tf_broad.sendTransform((.05,-.03,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G0", "EE1")
        self.tf_broad.sendTransform((.05,-.01,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G1", "EE1")
        self.tf_broad.sendTransform((.05,.01,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G2", "EE1")
        self.tf_broad.sendTransform((.05,.03,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G3", "EE1") 

        # End Effector 2 -> Each Gripper, G0 is the far left   
        self.tf_broad.sendTransform((.05,-.03,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G0", "EE2")
        self.tf_broad.sendTransform((.05,-.01,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G1", "EE2")
        self.tf_broad.sendTransform((.05,.01,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G2", "EE2")
        self.tf_broad.sendTransform((.05,.03,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G3", "EE2")

        # base_link -> Each LIDAR
        self.tf_broad.sendTransform((0, .125368, -.03914),  
                        tf.transformations.quaternion_from_euler(0, 3.14159, -1.570796),
                        rospy.Time.now(), "laser_left", "base_link")
        self.tf_broad.sendTransform((.08, 0, -.03914),  
                        tf.transformations.quaternion_from_euler(3.14159, 0, 0),
                        rospy.Time.now(), "laser_middle", "base_link")
        self.tf_broad.sendTransform((0, -.125368, -.03914),  
                        tf.transformations.quaternion_from_euler(0, 3.14159, 1.570796),
                        rospy.Time.now(), "laser_right", "base_link")
        self.tf_broad.sendTransform((0,0,-.03914), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "laser_comb", "base_link")

        # base_link -> main camera
        self.tf_broad.sendTransform((.1524,0,.1524), 
                        tf.transformations.quaternion_from_euler(1.57,3.1415,1.57),
                        rospy.Time.now(), "cam_0", "base_link")

        # base_link -> imu
        self.tf_broad.sendTransform((0,0,0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "imu", "base_link")

        # TEMP
        self.tf_broad.sendTransform((.15,.15,.15), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "EE1", "base_link")
        self.tf_broad.sendTransform((0,-.15,.15), 
                        tf.transformations.quaternion_from_euler(0,0,-1.57),
                        rospy.Time.now(), "EE2", "base_link")        
        #self.tf_broad.sendTransform((.2,.2,0), 
        #                 tf.transformations.quaternion_from_euler(0,0,1.57),
        #                 rospy.Time.now(), "base_link", "map") 

if __name__ == "__main__":
    rospy.init_node('tf_broadcaster')
    # Maybe add a way to force a publish of TF
    TFPublisher(10)

