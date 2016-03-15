#!/usr/bin/python
import rospy

from sensor_msgs.msg import MagneticField

import numpy as np
import matplotlib.pyplot as plt



class Calibrate():
    def __init__(self, topic_name):
        rospy.Subscriber(topic_name, MagneticField, self.got_mag_field)
        self.mag_field = None

    def got_mag_field(self, msg):
        # Get the field then make it a reasonable number. Not sure of units right now.
        self.mag_field = np.array(msg.magnetic_field)/1000.0


if __name__ == "__main__":
    rospy.init_node("magnetic_calibrator")
    c = Calibrate("mag")
    