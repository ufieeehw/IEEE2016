#!/usr/bin/python
import rospy

from sensor_msgs.msg import MagneticField

import numpy as np
import matplotlib.pyplot as plt

class Calibrate():
    def __init__(self, topic_name):
        self.mag_sub = rospy.Subscriber(topic_name, MagneticField, self.got_mag_field)
        self.mag_field = None

        self.x = np.array([])
        self.y = np.array([])

    def got_mag_field(self, msg):
        # Get the field then make it a reasonable number. Not sure of units right now.
        self.mag_field = np.array([msg.magnetic_field.x,msg.magnetic_field.y,msg.magnetic_field.z])/100.0
        # if self.mag_field[0] < 0:
        #     self.mag_field[0] += .32254*2
        self.x = np.append(self.x,self.mag_field[0])
        self.y = np.append(self.y,self.mag_field[1])
        print len(self.x),self.mag_field
        if len(self.x) == 200:
            self.done_caching()

    def done_caching(self):
        plt.scatter(self.x, self.y)
        centroid = [np.mean(self.x),np.mean(self.y)]
        plt.plot(centroid[0], centroid[1], 'ro')
        plt.show()
        self.mag_sub.unregister()
        rospy.signal_shutdown("done")

if __name__ == "__main__":
    rospy.init_node("magnetic_calibrator")
    c = Calibrate("mag")

    rospy.spin()
