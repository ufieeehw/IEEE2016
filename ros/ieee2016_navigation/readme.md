Navigation
=========

This package handles all of the navigation and localization nodes for Shia.

#Particle Filter
A [particle filter](https://en.wikipedia.org/wiki/Particle_filter) is a way of estimating our current location using sensor data. The reason we are going to use a particle filter rather then SLAM is because the LIDAR data is not accurate due to the dark walls of the field. The particle filter will account for this by estimating the *best* position (not nessicarily the 100% correct position). Currently, the particle filter takes in odometry data and publishes an esitmated pose as well as a `map -> base_link` transform.

###LIDAR
The particle filter relies on LIDAR data in order to localize. To activate the LIDAR, run ```roslaunch ieee2016_navigation laser.launch```. This will also launch the laser combiner script. This script takes the data from the three seperate LIDARs and combines it into one large scan. This combined scan is then passed to the particle filter for processing on. Running this scripts assumes the LIDAR udev rules are set up (see the [udev folder](https://github.com/ufieeehw/IEEE2016/tree/master/udev)). Sometimes the LIDARs wont start, so you'll need to unplug and plug them in (someone should figure out why this is).

###GPGPU
This implementation of the particle fiter uses pyopencl for [General Purpose computing on Graphics Processing Units ](https://en.wikipedia.org/wiki/General-purpose_computing_on_graphics_processing_units), in other words the filter does most of its processing on the graphics processor rather then on the CPU. This allows for much more effiencent execution of the particle filter. For more information on how this works or how to get it installed, see Matthew Langford.

#EKF
We use an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) to integrate odometry data from the wheel encoders and IMU data (currently only rotational data since the linear calibration was returning weird results) to generate a pose esitmation that gets fed into the particle filter. We are using the EKF provided in the [robot_localization](http://wiki.ros.org/robot_localization) ros package.
