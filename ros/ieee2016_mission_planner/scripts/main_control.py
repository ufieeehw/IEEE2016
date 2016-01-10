#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool, Int8

"""
For map version one

1. move forward until you get close to the wall
2. move left until vision detects QR codes of B blocks
3. wait until full robot
4. move left until near B blocks
5. detect box colors, and save box locations
6. move based on color of block
7. move back to position in (4) and go to (5) until blocks are gone
"""

class ShiaStateMachine():
    def __init__(self):
        self.current_state = 0
        self.map_version = 0

        self.running = False

        print " > State Machine Init Complete."

    def begin(self):
        # This will run the main loop
        print " > Running"
        print " > Map Version:",self.map_version

        current_state = self.current_state
        current_state += 1

        self.running = True
        while not rospy.is_shutdown():
            #print current_state
            if current_state == 1:
                self.drive_to_wall()
            elif current_state == 2:
                self.find_target_blocks()
            elif current_state == 3:
                self.load_robot()
            elif current_state == 4:
                self.move_to_train()
            elif current_state == 5:
                self.dectect_box_colors()
            elif current_state == 6:
                self.unload_blocks_by_color()
            elif current_state == 7:
                self.return_to_block()

            current_state += 1
            if current_state > 7: current_state = 0


    def drive_to_wall(self):
        # Turn on forward LIDAR to detect wall distance
        # Orient robot to so it is parallel with wall
        # Drive to set distance 
        
        print "1"
    def find_target_blocks(self):
        # Move 

        print "2"
    def load_robot(self):
        print "3"
    def move_to_train(self):
        print "4"
    def dectect_box_colors(self):
        print "5"
    def unload_blocks_by_color(self):
        print "6"
    def return_to_block(self):
        print "7"

class ros_manager():
    def __init__(self):
        self.state_machine = ShiaStateMachine()
        self.map_version_set = False

        # ROS inits
        rospy.init_node('main_control')
        rospy.Subscriber('start_command',Bool,self.recieve_start_command,self.state_machine)
        rospy.Subscriber('map_version',Int8,self.determine_map_version)

    def recieve_start_command(self,msg,s):
        if msg.data and not self.state_machine.running and self.map_version_set:
            print " > State Machine Starting..."
            self.state_machine.begin()

    def determine_map_version(self,msg):
        self.state_machine.map_version = msg.data
        self.map_version_set = True

        print " > Map Version:",self.state_machine.map_version


if __name__ == "__main__":
    ros_manager()
    rospy.spin()
