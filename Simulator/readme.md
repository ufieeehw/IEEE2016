Simulator

This was adapted from the 2015 robot. Original code writien by Jake Panikulam.

USAGE:

    roslaunch ieee2016_launch arm_sim.launch

SCRIPTS:

This script publishes test points in a circle to the '/sim/arm_des_pose' topic 
This is for simulation only. It ideally spins the links in circles.

    rosrun ieee2016_simulator arm_controller_test.py



   TODO:
   -Add Z direction movement
   -Add second arm
   -Indicate link interference
   -Add grippers and environment
