import numpy as np
import copy

class Controller():
    def __init__(self):
        # Set up interupts

        # PID Controllers for each motor
        p = 120
        i = 2950
        d = .0004
        temp_PID = PID(p,i,d)

        # List of PID controller for each motor
        self.PIDs = [copy.copy(temp_PID), copy.copy(temp_PID), copy.copy(temp_PID), copy.copy(temp_PID)]        

        # Counters for ticks 
        self.pid_ticks = [0,0,0,0]
        self.odom_ticks = [0,0,0,0]

        self.rads_per_tick = 2*np.pi / 4480.0

    def interupt(self, motor_index):
        '''
        Used to sum up the number of ticks
        '''
        self.ticks[motor_index] += 1

    def read_vel(self, motor_index, dt):
        '''
        Calculates velocity of motor based on the number of ticks moved since the last poll dt (s) ago.
        Resets the tick count at this point
        '''
        return self.ticks[motor_index] * self.rads_per_tick / dt

    def send_vel(self, velocity, portID):
        '''
        Send a velocity to some motor with portID.
        This function will do the PID mumbojumbo
        '''



class PID:
    """
    Discrete PID control stolen from the internet.
    """
    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

