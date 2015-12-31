Robot Diagnostics
=================


This package contains ROS tools for debugging the robot, automating out some of the tedium involved in test/debug.


# Manual Robot Control
Allows you to manually control the robot with a spacenav 3d mouse plugged into Shia.

    roslaunch ieee2016_diagnostics_ported base_spacenav_control.launch
    
If you want to run the spacenav node on a different computer (to remotely control Shia) tack ```remote:=true``` to the end. This also means the other computer will need to be connected to the same roscore and have a spacenav node running:

    rosrun spacenav_node spacenav_node
