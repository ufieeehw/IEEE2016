#!/usr/bin/python
from __future__ import division

import threading
import serial
import math
import yaml
import os
import numpy as np

import rospy
import rospkg
from std_msgs.msg import Header, Float64
from xmega_connector.msg import XMEGAPacket
from xmega_connector.srv import *
from geometry_msgs.msg import TwistStamped, Twist, Vector3, PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, \
                              TwistWithCovariance, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

from tf import transformations


rospack = rospkg.RosPack()
CALIBRATION_FILE_URI = os.path.join(rospack.get_path('ieee2016_xmega_connector_ported'), 'scripts/')

class XMEGAConnector(object):

    def __init__(self, port):
        self._serial = serial.Serial(port, 19200)

    def read_packet(self):
        rospy.logdebug("Reading packet from XMEGA")
        packet = XMEGAPacket()
        data = ''
        start_char_count = 0
        message_len = 0
        while True:
            if start_char_count < 3:
                if self._serial.read(1) == '^':
                    start_char_count += 1
                    rospy.logdebug("Reading from XMEGA - start character received (%d of 3)", start_char_count)
                    continue
            else:
                message_len = ord(self._serial.read(1))
                rospy.logdebug("Reading from XMEGA - packet length: %s", hex(message_len))
                data = self._serial.read(message_len)

                packet.header.stamp = rospy.Time.now()
                packet.msg_type = data[0]
                rospy.logdebug("Reading from XMEGA - packet type: %s", ord(packet.msg_type))
                packet.msg_length = message_len
                packet.msg_body = data[1:]
                rospy.logdebug("Reading from XMEGA - packet body: %s -- hex: %s", packet.msg_body, packet.msg_body.encode("hex"))
                return packet

    def send_packet(self, packet_to_send):
        rospy.logdebug("Sending packet to XMEGA")
        length = packet_to_send.msg_length
        type = packet_to_send.msg_type
        message = packet_to_send.msg_body

        self._serial.write("^^^")
        rospy.logdebug("Sending to XMEGA - sent 3 start characters to XMEGA")
        self._serial.write(chr(length))
        rospy.logdebug("Sending to XMEGA - length sent: %s", hex(length))
        self._serial.write(chr(type))
        rospy.logdebug("Sending to XMEGA - length sent: %s",  hex(type))
        self._serial.write(message)
        self._serial.write('\0') #need to send an additional character to get XMEGA to finish reading
        rospy.logdebug("Sending to XMEGA - message sent")

    def send_ack(self):
        ack_packet = XMEGAPacket()
        ack_packet.msg_type = 0x00
        ack_packet.msg_length = 0x01
        connector_object.send_packet(ack_packet)


class MagnetometerManager():
    '''
    This whole file is pretty messy, so I'm going to put this here so I don't have to 
    be annoyed with it.

    Deals with publishing Magnetometer date. I think this will be a pose with only rotation being changed.
    '''
    def __init__(self, service_manager, calibration_file_name = "calibration.yaml"):
        self.pose_est_pub = rospy.Publisher("/robot/navigation/mag_pose_vis", PoseStamped, queue_size=2) 
        self.p_c_s_est_pub = rospy.Publisher('/robot/navigation/mag_pose', PoseWithCovarianceStamped, queue_size=10)

        file_name = str(CALIBRATION_FILE_URI + calibration_file_name)
        with open(file_name, 'r') as infile:
            data = yaml.load(infile)
        
        self.correction_matrix = np.matrix(data['correction_matrix'])
        rospy.loginfo("Magnetometer calibration file loaded!")

        self.service_manager = service_manager

        # New service to get corrected heading
        rospy.Service('~get_heading_corrected', GetHeading, self.get_heading_service)


    def get_heading_service(self,srv):
        # Get the raw heading from the service (but do it locally, not over ros.)
        heading = self.service_manager.get_heading_service(None)

        xData,yData = self.correct_mag_data(heading.xData,heading.yData)[:2]

        service_response = GetHeadingResponse()
        service_response.xData = xData
        service_response.zData = 0
        service_response.yData = yData

        return service_response

    def publish_mag_data(self):
        # Get the raw heading from the service (but do it locally, not over ros.)
        heading = self.service_manager.get_heading_service(None)
        corrected_point = self.correct_mag_data(heading.xData,heading.yData)
        #print (corrected_point).astype(np.int32)
        angle = np.arctan2(corrected_point[1],corrected_point[0])
        self.generate_pose(angle)

    def correct_mag_data(self,xData,yData):
        # Take the corrective matrix and adjust the measured point with it
        point = np.array([[xData],[yData],[1]])
        corrected_point = np.dot(self.correction_matrix,point)
        return corrected_point
        
    def generate_pose(self, angle):
        # Generate a pose, all values but the yaw will be 0.
        q = transformations.quaternion_from_euler(0, 0, angle)
        header = Header(
            stamp=rospy.Time.now(),
            frame_id="map"
        )
        pose = Pose(
            position=Point(x=0, y=0, z=0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )

        # Publish pose stamped - just for displaying in rviz
        self.pose_est_pub.publish(
            PoseStamped(
                header=header,
                pose=pose
            )
        )

        # Publish pose with covariance stamped.
        p_c_s = PoseWithCovarianceStamped()
        p_c = PoseWithCovariance()
        covariance = np.array([.05,   0,   0,   0,   0,   0,
                                 0, .05,   0,   0,   0,   0,
                                 0,   0, .05,   0,   0,   0,
                                 0,   0,   0, .05,   0,   0,
                                 0,   0,   0,   0, .05,   0,
                                 0,   0,   0,   0,   0, .05])**2
        p_c.pose = pose
        p_c.covariance = covariance
        p_c_s.header = header
        p_c_s.pose = p_c
        # Publish pose estimation
        self.p_c_s_est_pub.publish(p_c_s)


class ServiceManager():
    def __init__(self, connector_object, xmega_lock):
        # Initialize xmega services
        rospy.Service('~echo', Echo, self.echo_service)
        rospy.Service('~set_wheel_speeds', SetWheelSpeeds, self.set_wheel_speed_service)
        rospy.Service('~get_odometry', GetOdometry, self.get_odometry_service)
        rospy.Service('~get_heading', GetHeading, self.get_heading_service)
        rospy.Service('~get_motion', GetMotion, self.get_motion_service)
        
        self.connector_object = connector_object
        self.xmega_lock = xmega_lock

    def echo_service(self, echo_request):
        self.xmega_lock.acquire(True)  # wait until lock can be acquired before proceeding
        rospy.loginfo("XMEGA echo - about to echo: %s", echo_request.send)

        packet = XMEGAPacket()
        packet.msg_body = echo_request.send
        packet.msg_type = 0x02  # 0x02 echo request, 0x03 echo reply
        packet.msg_length = len(packet.msg_body) + 1

        self.connector_object.send_packet(packet)
        rospy.loginfo("XMEGA echo - sent echo request packet")
        response_packet = self.connector_object.read_packet()
        rospy.loginfo("XMEGA echo - received echo response packet")

        rospy.loginfo("XMEGA echo - sending ack packet")
        self.connector_object.send_ack()
        rospy.loginfo("XMEGA echo - sent ack packet")

        service_response = EchoResponse()
        service_response.recv = response_packet.msg_body
        rospy.loginfo("XMEGA echo - received response: %s", service_response.recv)
        self.xmega_lock.release()
        return service_response


    def set_wheel_speed_service(self, ws_req):
        self.xmega_lock.acquire(True)
        packet = XMEGAPacket()
        packet.msg_type = 0x04

        wheel1 = int(ws_req.wheel1 * 1000.0)
        wheel2 = int(ws_req.wheel2 * 1000.0)
        wheel3 = int(ws_req.wheel3 * 1000.0)
        wheel4 = int(ws_req.wheel4 * 1000.0)

        packet.msg_body = struct.pack('<llll', wheel1, wheel2, wheel3, wheel4)
        packet.msg_length = len(packet.msg_body) + 1

        self.connector_object.send_packet(packet)

        self.xmega_lock.release()
        return SetWheelSpeedsResponse()


    def get_odometry_service(self, odo_req):
        self.xmega_lock.acquire(True)
        packet = XMEGAPacket()
        packet.msg_type = 0x05
        packet.msg_length = 1

        self.connector_object.send_packet(packet)

        response_packet = self.connector_object.read_packet()
        wheel1, wheel2, wheel3, wheel4 = struct.unpack("<iiii", response_packet.msg_body)
        self.connector_object.send_ack()

        service_response = GetOdometryResponse()
        service_response.wheel1 = wheel1 / 1000.
        service_response.wheel2 = wheel2 / 1000.
        service_response.wheel3 = wheel3 / 1000.
        service_response.wheel4 = wheel4 / 1000.
        self.xmega_lock.release()

        return service_response

    def get_heading_service(self, head_req):
        self.xmega_lock.acquire(True)
        packet = XMEGAPacket()
        packet.msg_type = 0x0A
        packet.msg_length = 1

        self.connector_object.send_packet(packet)

        response_packet = self.connector_object.read_packet()
        xData, zData, yData = struct.unpack("<hhh", response_packet.msg_body)
        self.connector_object.send_ack()

        service_response = GetHeadingResponse()
        service_response.xData = xData
        service_response.zData = zData
        service_response.yData = yData

        self.xmega_lock.release()
        return service_response

    def get_motion_service(self, m_req):
        self.xmega_lock.acquire(True)
        print("Lock aquired")
        packet = XMEGAPacket()
        packet.msg_type = 0x0B
        packet.msg_length = 1

        print("Sending message")
        self.connector_object.send_packet(packet)

        response_packet = self.connector_object.read_packet()
        xAccelData, yAccelData, zAccelData, xGyroData, yGyroData, zGyroData = struct.unpack("<hhhhhh", response_packet.msg_body)
        self.connector_object.send_ack()

        print( "Creating response")
        service_response = GetMotionResponse()
        service_response.xAccelData = xAccelData
        service_response.yAccelData = yAccelData
        service_response.zAccelData = zAccelData
        service_response.xGyroData = xGyroData
        service_response.yGyroData = yGyroData
        service_response.zGyroData = zGyroData

        print( "Returning response")
        self.xmega_lock.release()
        return service_response


if __name__ == "__main__":

    rospy.init_node('xmega_connector')#, log_level=rospy.DEBUG)

    connector_object = XMEGAConnector(rospy.get_param('~port'))
    xmega_lock = threading.Lock()

    # Define objects
    xmega_services = ServiceManager(connector_object,xmega_lock)
    magnetometer = MagnetometerManager(xmega_services)

    rate = rospy.Rate(10) #hz
    while not rospy.is_shutdown():
        rate.sleep()
        magnetometer.publish_mag_data()
