#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class TFPublisher():
    def __init__(self,rate):
        self.tf_broad = tf.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber("/robot/odom", Odometry, self.got_odom, queue_size=2)
        self.elevator_sub = rospy.Subscriber("/robot/arms/elevator", Float64, self.got_elevator_pos, queue_size=2)
        self.linear_rail_sub = rospy.Subscriber("/robot/arms/rail", Float64, self.got_linear_rail_pos, queue_size=2)
        
        # base_link (0,0,0) = center base(top level) of Shia

        self.elevator_pos = .1
        self.linear_rail_pos = 0

        r = rospy.Rate(rate) #hz
        while not rospy.is_shutdown():
            #print "Transforming..."
            # Publish all of our static transformations
            self.static_tf()

            r.sleep()

    def got_elevator_pos(self,msg):
        self.elevator_pos = msg.data
        print self.elevator_pos

    def got_linear_rail_pos(self,msg): 
        self.linear_rail_pos = msg.data

    def got_odom(self,msg):
        msg = msg.pose.pose
        self.tf_broad.sendTransform((msg.position.x,msg.position.y,0),(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),rospy.Time.now(),"odom_scan","map")

    def static_tf(self):
        # End Effector 1 -> Each Gripper, G0 is the far left
        self.tf_broad.sendTransform((.10922,-.09525,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G0", "EE1")
        self.tf_broad.sendTransform((.10922,-.03175,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G1", "EE1")
        self.tf_broad.sendTransform((.10922,.03175,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G2", "EE1")
        self.tf_broad.sendTransform((.10922,.09525,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "1-G3", "EE1") 

        # End Effector 2 -> Each Gripper, G0 is the far left   
        self.tf_broad.sendTransform((.10922,-.09525,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G3", "EE2")
        self.tf_broad.sendTransform((.10922,-.03175,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G2", "EE2")
        self.tf_broad.sendTransform((.10922,.03175,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G1", "EE2")
        self.tf_broad.sendTransform((.10922,.09525,.03302), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "2-G0", "EE2")

        # base_link -> Each LIDAR
        self.tf_broad.sendTransform((0, .125368, -.0775),  
                        tf.transformations.quaternion_from_euler(0, 3.14159, -1.590796),
                        rospy.Time.now(), "laser_left", "base_link")
        self.tf_broad.sendTransform((.1, 0, -.0775),  
                        tf.transformations.quaternion_from_euler(3.14159, 0, 0),
                        rospy.Time.now(), "laser_front", "base_link")
        self.tf_broad.sendTransform((0, -.125368, -.0775),  
                        tf.transformations.quaternion_from_euler(0, 3.14159, 1.570796),
                        rospy.Time.now(), "laser_right", "base_link")
        self.tf_broad.sendTransform((-.1, 0, -.0775),  
                        tf.transformations.quaternion_from_euler(3.14159, 0, 3.14159),
                        rospy.Time.now(), "laser_back", "base_link")
        
        self.tf_broad.sendTransform((0,0,-.0775), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "laser_fused", "base_link")

        # elevator -> main cameras
        self.tf_broad.sendTransform((.072,.0584,.0354), 
                        tf.transformations.quaternion_from_euler(1.57,3.1415,1.57),
                        rospy.Time.now(), "cam_1_vision", "elevator")
        self.tf_broad.sendTransform((-.072,-.0584,.0354), 
                        tf.transformations.quaternion_from_euler(1.57,3.1415,-1.57),
                        rospy.Time.now(), "cam_2_vision", "elevator")
        self.tf_broad.sendTransform((.072,.0584,.0354), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(), "cam_1_pose", "elevator")
        self.tf_broad.sendTransform((-.072,-.0584,.0354), 
                        tf.transformations.quaternion_from_euler(0,0,3.1415),
                        rospy.Time.now(), "cam_2_pose", "elevator")

        # # base_link -> imu
        # self.tf_broad.sendTransform((0,0,0), 
        #                 tf.transformations.quaternion_from_euler(0,0,0),
        #                 rospy.Time.now(), "imu", "base_link")

        # base_link -> elevator 
        self.tf_broad.sendTransform((0,0,.07112), 
                tf.transformations.quaternion_from_euler(0,0,1.5707),
                rospy.Time.now(), "elevator", "base_link")

        # elevator -> Each End Effector
        self.tf_broad.sendTransform((0,.02667,-.06925), 
                tf.transformations.quaternion_from_euler(0,0,0),
                rospy.Time.now(), "EE1", "elevator")
        self.tf_broad.sendTransform((0,-.02667,-.06925), 
                tf.transformations.quaternion_from_euler(0,0,3.1416),
                rospy.Time.now(), "EE2", "elevator")


        self.tf_broad.sendTransform((0,0,.127), 
                tf.transformations.quaternion_from_euler(0,0,0),
                rospy.Time.now(), "base_link", "base_footprint")

        #Temp
        self.tf_broad.sendTransform((0,0,0), 
                tf.transformations.quaternion_from_euler(0,0,0),
                rospy.Time.now(), "base_footprint", "odom")

        # self.tf_broad.sendTransform((.47457,1.629,0), 
        #         (0,0,.9999,-.0146),
        #         rospy.Time.now(), "base_footprint", "map")

if __name__ == "__main__":
    rospy.init_node('tf_broadcaster')
    # Maybe add a way to force a publish of TF
    TFPublisher(30)

