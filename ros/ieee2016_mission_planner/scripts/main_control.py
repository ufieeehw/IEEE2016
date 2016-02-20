#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int8, Header
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
from ieee2016_msgs.srv import NavWaypoint, ArmWaypoint
import tf

from waypoint_utils import load_waypoints
import time
import os

class ShiaStateMachine():
    def __init__(self):
        self.ros_manager = ros_manager(self)

        self.current_state = 0
        self.map_version = 0

        self.running = False
        self.waypoints = load_waypoints()

        print "> State Machine Init Complete."
        print "> Waiting for map selection and start command."

        rospy.spin()

    def begin(self):
        # This will run the main loop
        print
        print "> Running"
        print "> Map Version:",self.map_version
        print

        self.current_state+=1

        self.running = True
        rate = rospy.Rate(25) #hz
        while not rospy.is_shutdown():
            
            self.ros_manager.publish_current_state(self.current_state)

            if self.current_state == 1:
                """
                The goals of this state are to drive through the box, then rotate in order to get camera 1 in view of the blocks
                """
                print "=== STATE 1 ==="
                self.ros_manager.set_waypoint(self.waypoints['through_box'])
                self.ros_manager.set_waypoint(self.waypoints['examine_blocksC'])
                self.current_state+=1

            elif self.current_state == 2:
                """
                The goals of this state is to generate a point cloud of blocks for the C blocks
                """
                print "=== STATE 2 ==="
                
            # elif self.current_state == 3:
            #     self.load_robot()
            # elif self.current_state == 4:
            #     self.move_to_train()
            # elif self.current_state == 5:
            #     self.dectect_box_colors()
            # elif self.current_state == 6:
            #     self.unload_blocks_by_color()
            # elif self.current_state == 7:
            #     self.return_to_block()

            if self.current_state > 7: self.current_state = 0
            rate.sleep()

class ros_manager():
    def __init__(self,s):
        self.state_machine = s
        self.map_version_set = False

        # ROS inits
        self.nav_waypoint_pub = rospy.Publisher("/robot/waypoint", PoseStamped, queue_size=1)
        self.state_pub = rospy.Publisher("/robot/current_state", Int8, queue_size=10)
        self.nav_waypoint = rospy.ServiceProxy('/robot/nav_waypoint', NavWaypoint)
        
        rospy.init_node('main_control')
        
        rospy.Subscriber('/settings/start_command',Bool,self.recieve_start_command,self.state_machine)
        rospy.Subscriber('/settings/map_version',Int8,self.determine_map_version)
        

    def recieve_start_command(self,msg,s):
        if msg.data and not self.state_machine.running and self.map_version_set:
            print "> State Machine Starting..."
            self.state_machine.begin()

    def determine_map_version(self,msg):
        self.state_machine.map_version = msg.data
        self.map_version_set = True

        print "> Map Version:",self.state_machine.map_version

    def set_waypoint(self,waypoint):
        q = tf.transformations.quaternion_from_euler(0, 0, waypoint[2])
        p_s = PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map"
                ),
                pose=Pose(
                    position=Point(
                        x=waypoint[0],
                        y=waypoint[1],
                        z=0
                    ),
                    orientation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3],
                    )
                )
            )
        self.nav_waypoint_pub.publish(p_s)
        print "> Waypoint published:",waypoint
        print "> Moving..."
        success = self.nav_waypoint(p_s)
        print "> Movement Complete!"

    def publish_current_state(self,state):
        self.state_pub.publish(state)

if __name__ == "__main__":
    os.system("clear")
    ShiaStateMachine()
