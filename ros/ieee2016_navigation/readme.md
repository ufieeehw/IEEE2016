SLAM
====

Currently, this package handles activating LIDARS with appropriate tf frames, fusing LIDAR data into one large scan, and dealing with the [Hector Mapping](http://wiki.ros.org/hector_mapping) package.

#How to Run
First, make sure you have the hector_mapping and hokuyo_node packages installed, if you do not, run 

    sudo apt-get install ros-jade-hector-mapping ros-jade-hokuyo-node
Next you can start the LIDAR modules and begin publishing each modules' tf frame.

    roslaunch ieee2016_navigation laser.launch
This also runs the script responsible for fusing multiple LIDAR scans into one big scan: ```laser_comb.py```.

Finally, you can run the SLAM launch file.

    roslaunch ieee2016_navigation nav_dat_bitch.launch

#Viewing Data
Currently, I have only used RVIZ to vizualize all of this data. An occupancy grid is published to the ```/map``` topic, a pose estimation to ```/slam_out_pose```, and 4 LaserScan messages to various topics (left, middle, right, and combined). Hector_mapping gives a tf link between ```map``` and ```base_link``` (in the future when more postition sensors are available I suspect that may change). Other tf links provided by these nodes are ```base_link -> laser``` (for each laser including the combined LaserScan)

#Troubleshooting
* For some reason you have to unplug the LIDAR usb hub and replug it in after you restart Shia. Someone should figure out why this is and fix it.
* Also, if you're not running this on Shia, you'll want to make sure you have your udev definitions set up right. For info on how to do this refer to the [udev folder](https://github.com/ufieeehw/IEEE2016/tree/master/udev).
* SLAM has trouble dealing with flat walls that don't change and potentionally has problems if the area changes too much (moving walls or people moving in the frame). There's not really a fix for this right now but just so you know.

#Future
* Everthing will be chrome.
* Integrating more navigation sensors (IMU, encoder, VSLAM?) to create a more accurate pose estimation that can deal with flat walls. To accomlish this, we will probably use the [```robot_localization```](http://wiki.ros.org/robot_localization) ros package.
* Fix various bugs
