Shia Controllers
===================

Nodes that mid and high-level control for Shia. If you're curious about implementation, they are well-commented. For questions, check the git-blame for who to ask.


# Arm Controller
Given a desired position in space, computes the servo angles to achieve that position

## Usage
    rosrun ieee2016_controller arm_controller.py
Publish to ```/arm_des_pose```
With message type ```PointStamped```


## Requires
NOT IMPLEMENTED=>Servos plugged in and properly limited, but works with simulation
Works with pygame simulation. See ieee2016_simulator package README for usage.

# Mecanum Controller
Given a desired linear and angular velocity for Shia, compute the individual wheel speeds to achieve those, while minimizing the sum of the squares of the wheel velocities.

The robot is overactuated (4 independent actuators on 3 degrees of freedom: yaw, x-translation and y-translation). As such, you have four equations in three variables, for converting a desired Twist to wheel actions. We solve this with linear least squares.


## Usage
    rosrun ieee2016_controller mecanum_controller.py
Publish to ```/twist```
With message type ```TwistStamped```

## Requires



