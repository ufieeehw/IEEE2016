#!/usr/bin/python
from __future__ import division

import rospy
import rospkg

from sensor_msgs.msg import MagneticField
from ieee2016_xmega_connector_ported.srv import GetHeading

import numpy as np
import matplotlib.pyplot as plt
import yaml
import os
from scipy import optimize
from numpy.linalg import eig, inv

rospack = rospkg.RosPack()
CALIBRATION_FILE_URI = os.path.join(rospack.get_path('ieee2016_xmega_connector_ported'), 'scripts/')

# Shamelessly taken from online guide ===========================
def fitEllipse(x,y):
    x = x[:,np.newaxis]
    y = y[:,np.newaxis]
    D =  np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
    S = np.dot(D.T,D)
    C = np.zeros([6,6])
    C[0,2] = C[2,0] = 2; C[1,1] = -1
    E, V =  eig(np.dot(inv(S), C))
    n = np.argmax(np.abs(E))
    a = V[:,n]
    return a

def ellipse_center(a):
    b,c,d,f,g,a = a[1]/2.0, a[2], a[3]/2.0, a[4]/2.0, a[5], a[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return np.array([x0,y0])

def ellipse_angle_of_rotation(a):
    b,c,d,f,g,a = a[1]/2.0, a[2], a[3]/2.0, a[4]/2.0, a[5], a[0]
    if b == 0:
        if a > c:
            return 0
        else:
            return np.pi/2
    else:
        if a > c:
            return np.arctan(2*b/(a-c))/2
        else:
            return np.pi/2 + np.arctan(2*b/(a-c))/2

def ellipse_axis_length(a):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    up = 2.0*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
    down1=(b*b-a*c)*( (c-a)*np.sqrt(1.0+4*b*b/((a-c)*(a-c)))-(c+a))
    down2=(b*b-a*c)*( (a-c)*np.sqrt(1.0+4*b*b/((a-c)*(a-c)))-(c+a))
    res1=np.sqrt(up/down1)
    res2=np.sqrt(up/down2)
    return np.array([res1, res2])
# Back to non stolen code =======================================

class Calibrate():
    def __init__(self, topic_name, cache_length, calibration_file_name = "calibration.yaml"):
        self.mag_proxy = rospy.ServiceProxy('/robot/xmega_connector/get_heading', GetHeading)
        self.mag_field = None

        self.cache_length = cache_length
        self.calibration_file_name = calibration_file_name

        self.x = np.array([])
        self.y = np.array([])
        plt.ion()
        self.get_mag_field()

    def get_mag_field(self):
        # Get the field then make it a reasonable number. Not sure of units right now.
        print "Running"
        while len(self.x) < self.cache_length:
            print "Running..."
            magnetic_field = self.mag_proxy()
            self.mag_field = np.array([magnetic_field.xData,magnetic_field.yData,magnetic_field.zData])
            
            print len(self.x),self.mag_field
            self.x = np.append(self.x,self.mag_field[0])
            self.y = np.append(self.y,self.mag_field[1])
            plt.scatter(self.x, self.y)
            plt.show()
            plt.pause(.01)
            

        # Once we have saved 'cache_length' many points, start calibrating
        self.done_caching()

    def done_caching(self):
        # Once we are done caching, display the uncalibrated data and start the calibration.
        plt.scatter(self.x, self.y)
        centroid = [np.mean(self.x),np.mean(self.y)]
        plt.plot(centroid[0], centroid[1], 'ro')
        plt.show()
        self.generate_correction_matrix()

    def generate_correction_matrix(self):
        '''
        Generate ellipse through given points (using internet code).
        The generated ellipse will have a center position, a major and minor axis, and an offset theta of the major axis from 0 rads.

        To correct we construct a transformation matrix to offset the raw ellipse to the origin, rotate it so that the angle between the 
        major axis and the positive x axis is 0, then scale the x values so that the length of the major axis is the same as the minor axis.
        '''

        a = fitEllipse(self.x,self.y)
        xc,yc = ellipse_center(a)
        theta = ellipse_angle_of_rotation(a)
        axis_len = ellipse_axis_length(a)
        s = axis_len[0]/axis_len[1]

        x_deviation = np.sqrt(sum((self.x-xc)**2)/len(self.x))
        y_deviation = np.sqrt(sum((self.y-yc)**2)/len(self.y))
        print "Calibration Results:"
        print "========================================================="
        print "           Old Center at:",xc,yc
        print "               Old Angle:",theta
        print " Old Maj/Min Axis Length:",axis_len
        print "         Old X Deviation:",x_deviation
        print "         Old Y Deviation:",y_deviation
        print "========================================================="

        # Generate the transformation matrix. Translate -> Rotate -> Scale
        iron_matrix = np.array([
            [  s*np.cos(theta), s*np.sin(theta), -xc*s*np.cos(theta)-yc*s*np.sin(theta)],
            [   -np.sin(theta),   np.cos(theta),      xc*np.sin(theta)-yc*np.cos(theta)],
            [                0,               0,                                      1]])

        print "Corrective Matrix:"
        print iron_matrix

        pre_corrected = {
            'xc' : float(xc),
            'yc' : float(yc),
            'theta' : float(theta),
            'major_len' : float(axis_len[0]),
            'minor_len' : float(axis_len[1]),
            'x_dev' : float(x_deviation),
            'y_dev': float(y_deviation)
        }

        # The rest is not nessicary for calibration but is used to display the new calibrated info.
        points = np.vstack((self.x,self.y,np.full(len(self.x),1,dtype=np.int32)))
        corrected_points = np.dot(iron_matrix,points)

        a = fitEllipse(*corrected_points[:2])
        center = ellipse_center(a)
        theta = ellipse_angle_of_rotation(a)
        axis_len = ellipse_axis_length(a)

        x_deviation = np.sqrt(sum((corrected_points[0])**2)/len(corrected_points[0]))
        y_deviation = np.sqrt(sum((corrected_points[1])**2)/len(corrected_points[1]))

        # Quick note: the center should be very close to the origin, the maj and min axis lengths should be
        # very similar, the deviations should be very close to 1, but the angle doesn't need to be 0. This is
        # due to the fact that if the lenghts of maj=min, then we have a circle and therefore no calculatable
        # angle offset from the x-axis.
        print "========================================================="
        print "           New Center at:",xc,yc
        print "              New Angle*:",theta
        print " New Maj/Min Axis Length:",axis_len
        print "         New X Deviation:",x_deviation
        print "         New Y Deviation:",y_deviation
        print "========================================================="
        print 

        post_corrected = {
            'xc' : float(center[0]),
            'yc' : float(center[1]),
            'theta' : float(theta),
            'major_len' : float(axis_len[0]),
            'minor_len' : float(axis_len[1]),
            'x_dev' : float(x_deviation),
            'y_dev': float(y_deviation)
        }

        # Print points, mostly for trouble shooting.
        print "Old points:"
        for i in range(len(self.x)):
            print "(%.4f,%.4f),"%(self.x[i],self.y[i]),
        print
        print "New points:"
        for i in range(len(self.x)):
            print "(%.4f,%.4f),"%(corrected_points[0][i],corrected_points[1][i]),
        print

        plt.scatter(corrected_points[0], corrected_points[1])
        centroid = [np.mean(corrected_points[0]),np.mean(corrected_points[1])]
        plt.plot(centroid[0], centroid[1], 'ro')
        #plt.show()

        # Write the calibration data, duh.
        print "Writing calibration data..."
        details = {'pre_corrected':pre_corrected,'post_corrected':post_corrected}
        self.write_to_file(iron_matrix,details)

    def write_to_file(self, matrix, details):
        details['correction_matrix'] = matrix.tolist()
        print details['correction_matrix']
        file_name = str(CALIBRATION_FILE_URI + self.calibration_file_name)

        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(details, default_flow_style=False))

        print
        print "Calibration file: %s saved!" % self.calibration_file_name

        # Exit program when we are done
        rospy.signal_shutdown("Finished Calibration.")
    
    def test_load(self):
        '''
        Just for testing the output of the yaml file - this can be removed later.
        '''
        file_name = str(CALIBRATION_FILE_URI + self.calibration_file_name)
        with open(file_name, 'r') as infile:
            data = yaml.load(infile)
        print np.matrix(data['correction_matrix'])

if __name__ == "__main__":
    rospy.init_node("magnetic_calibrator")
    c = Calibrate("mag",100)
    rospy.spin()
