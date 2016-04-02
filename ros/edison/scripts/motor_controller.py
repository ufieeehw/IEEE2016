#!/usr/bin/env python
import rospy

from ieee2016_msgs.srv import GetOdometry, SetWheelSpeeds

import numpy as np
import copy
import time

# Variables we need everywhere.
P = 120
I = 2950
D = .0004
RADS_PER_TICK = 2*np.pi / 4480.0

class Motor():
    def __init__(self, port_id, name = None):
        self.name = name
        self.port_id = port_id
        self.pid_ticks = 0
        self.odom_ticks = 0

        self.last_pid_time = None
        self.last_odom_time = None

        self.PID = PID(P,I,D)
    
    def get_vel(self, odom = False):
        '''
        Reads the number of ticks since the last polling then divides it by the time since last poll: dt (s) to get
        'instantaneous' rads/s.

        Odom specifies wether we are polling for odom purposes for internal PID reasons (there are two counters).
        '''
        if odom:
            # If it is the first poll, set the inital start time.
            if self.last_odom_time is None: 
                self.last_odom_time = time.time()
                return 0

            # Calculate velocity then reset relevant settings.
            vel = self.odom_ticks * RADS_PER_TICK / (time.time() - self.last_odom_time)
            self.last_odom_time = time.time()
            motor.odom_ticks = 0

        else:
            # If it is the first poll, set the inital start time.
            if self.last_pid_time is None: 
                self.last_pid_time = time.time()
                return 0
            
            # Calculate velocity then reset relevant settings.
            vel = self.pid_ticks * RADS_PER_TICK / (time.time() - self.last_pid_time)
            self.last_pid_time = time.time()
            motor.pid_ticks = 0

        return vel

    def set_vel(self, velocity):
        '''
        Sends some velocity to this motor.

        NEEDS IMPLEMENTATION
        '''

    def set_target_velcity(self, target_velocity):
        '''
        Send a desired velocity to this motor.
        '''
        # Tell the PID controller the target velocity
        self.PID.setpoint(target_velocity)

    def interupt_trigger(self):
        '''
        Callback when a interupt is triggered.

        NEEDS IMPLEMENTATION
        '''
        self.pid_ticks += 1
        self.odom_ticks += 1

class Controller():
    def __init__(self, *motors):
        # Set up interupts here to modify that motors tick numbers
        # IDK WHAT GOES HERE

        # Lists the motors
        self.motors = motors

        rospy.Service('/robot/edison/get_odometry', GetOdometry, self.get_odom_callback)
        rospy.Service('/robot/edison/set_wheel_speeds', SetWheelSpeeds, self.set_speeds_callback)

    def run(self):
        '''
        For each motor:
            Gets the current velocity of motor.
            Calculates the new PID'd velocity.
            Sends the corrected velocity to the motor.

        All velocities should be in rads/sec (so they are really angular velocities).
        '''
        r = rate(100) #hz
        while not rospy.is_shutdown():
            for motor in self.motors:
                current_vel = motor.get_vel()
                new_vel = motor.PID.update(current_vel)
                motor.set_vel(new_vel)
            r.sleep()

    def set_speeds(self, srv):
        '''
        Sets the target velocities for each wheel.
        '''
        # Doing it like this is faster then a for loop maybe?
        self.motors[0].set_target_velcity(srv.wheel1)
        self.motors[1].set_target_velcity(srv.wheel2)
        self.motors[2].set_target_velcity(srv.wheel3)
        self.motors[3].set_target_velcity(srv.wheel4)

    def get_odom(self, srv):
        '''
        Returns the velocity of each wheel to the service caller.
        '''
        return self.motors[0].get_vel(odom=True),
               self.motors[1].get_vel(odom=True),
               self.motors[2].get_vel(odom=True),
               self.motors[3].get_vel(odom=True) 

class PID():
    """
    Discrete PID control adpated from the internet.
    """
    def __init__(self, P_term=2.0, I_term=0.0, D_term=1.0, derviator=0, integrator=0, integrator_max=500, integrator_min=-500):
        self.Kp = P_term
        self.Ki = I_term
        self.Kd = D_term
        self.derviator = derviator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.derviator)
        self.derviator = self.error

        self.integrator = self.integrator + self.error

        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.I_value = self.integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def set_point(self,set_point):
        """
        Initilize the target of the PID controller.
        """
        self.set_point = set_point
        self.integrator = 0
        self.derviator = 0