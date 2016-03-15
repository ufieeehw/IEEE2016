#!/usr/bin/python

from __future__ import division

import threading
import serial
import rospy
import math

import numpy
from std_msgs.msg import Header, Float64
from xmega_connector.msg import XMEGAPacket
from xmega_connector.srv import *
from geometry_msgs.msg import TwistStamped, Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import MagneticField
from tf import transformations

rospy.init_node('xmega_connector')#, log_level=rospy.DEBUG)

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


connector_object = XMEGAConnector(rospy.get_param('~port'))

xmega_lock = threading.Lock()


def echo_service(echo_request):
    xmega_lock.acquire(True)  # wait until lock can be acquired before proceeding
    rospy.loginfo("XMEGA echo - about to echo: %s", echo_request.send)

    packet = XMEGAPacket()
    packet.msg_body = echo_request.send
    packet.msg_type = 0x02  # 0x02 echo request, 0x03 echo reply
    packet.msg_length = len(packet.msg_body) + 1

    connector_object.send_packet(packet)
    rospy.loginfo("XMEGA echo - sent echo request packet")
    response_packet = connector_object.read_packet()
    rospy.loginfo("XMEGA echo - received echo response packet")

    rospy.loginfo("XMEGA echo - sending ack packet")
    connector_object.send_ack()
    rospy.loginfo("XMEGA echo - sent ack packet")

    service_response = EchoResponse()
    service_response.recv = response_packet.msg_body
    rospy.loginfo("XMEGA echo - received response: %s", service_response.recv)
    xmega_lock.release()
    return service_response


def set_wheel_speed_service(ws_req):
    xmega_lock.acquire(True)
    packet = XMEGAPacket()
    packet.msg_type = 0x04

    wheel1 = int(ws_req.wheel1 * 1000.0)
    wheel2 = int(ws_req.wheel2 * 1000.0)
    wheel3 = int(ws_req.wheel3 * 1000.0)
    wheel4 = int(ws_req.wheel4 * 1000.0)

    packet.msg_body = struct.pack('<llll', wheel1, wheel2, wheel3, wheel4)
    packet.msg_length = len(packet.msg_body) + 1

    connector_object.send_packet(packet)

    xmega_lock.release()
    return SetWheelSpeedsResponse()


def get_odometry_service(odo_req):
    xmega_lock.acquire(True)
    packet = XMEGAPacket()
    packet.msg_type = 0x05
    packet.msg_length = 1

    connector_object.send_packet(packet)

    response_packet = connector_object.read_packet()
    wheel1, wheel2, wheel3, wheel4 = struct.unpack("<iiii", response_packet.msg_body)
    connector_object.send_ack()

    service_response = GetOdometryResponse()
    service_response.wheel1 = wheel1 / 1000.
    service_response.wheel2 = wheel2 / 1000.
    service_response.wheel3 = wheel3 / 1000.
    service_response.wheel4 = wheel4 / 1000.
    xmega_lock.release()

    return service_response

INCH = 25.4e-3
width =  9.345*INCH
length = 8.119*INCH

wheels = [
    ((-length/2, -width/2, 0), (+1, -1, 0)), # rear right
    ((-length/2, +width/2, 0), (+1, +1, 0)), # rear left
    ((+length/2, -width/2, 0), (+1, +1, 0)), # front right
    ((+length/2, +width/2, 0), (+1, -1, 0)), # front left
]

wheel_diameter = 54e-3 # 54 mm
wheel_radius = wheel_diameter / 2

def get_heading_service(head_req):
    xmega_lock.acquire(True)
    packet = XMEGAPacket()
    packet.msg_type = 0x0A
    packet.msg_length = 1

    connector_object.send_packet(packet)

    response_packet = connector_object.read_packet()
    xData, zData, yData = struct.unpack("<hhh", response_packet.msg_body)
    connector_object.send_ack()

    service_response = GetHeadingResponse()
    service_response.xData = xData
    service_response.zData = zData
    service_response.yData = yData

    xmega_lock.release()
    return service_response

def get_motion_service(m_req):
    xmega_lock.acquire(True)
    print("Lock aquired")
    packet = XMEGAPacket()
    packet.msg_type = 0x0B
    packet.msg_length = 1

    print("Sending message")
    connector_object.send_packet(packet)

    response_packet = connector_object.read_packet()
    xAccelData, yAccelData, zAccelData, xGyroData, yGyroData, zGyroData = struct.unpack("<hhhhhh", response_packet.msg_body)
    connector_object.send_ack()

    print( "Creating response")
    service_response = GetMotionResponse()
    service_response.xAccelData = xAccelData
    service_response.yAccelData = yAccelData
    service_response.zAccelData = zAccelData
    service_response.xGyroData = xGyroData
    service_response.yGyroData = yGyroData
    service_response.zGyroData = zGyroData

    print( "Returning response")
    xmega_lock.release()
    return service_response

xyz_array = lambda o: numpy.array([o.x, o.y, o.z])

rospy.Service('~echo', Echo, echo_service)
rospy.Service('~set_wheel_speeds', SetWheelSpeeds, set_wheel_speed_service)
rospy.Service('~get_odometry', GetOdometry, get_odometry_service)
rospy.Service('~get_heading', GetHeading, get_heading_service)
rospy.Service('~get_motion', GetMotion, get_motion_service)

heading_proxy = rospy.ServiceProxy('~get_heading', GetHeading)
# odom_pub = rospy.Publisher('odom', PoseStamped)
mag_pub = rospy.Publisher('mag', MagneticField, queue_size=1)

while not rospy.is_shutdown():
    rospy.sleep(rospy.Duration(0.1))
    # odom = get_odometry_service(None)
    stamp = rospy.Time.now()
    
    resp = heading_proxy()
    mx = resp.xData
    mz = resp.zData
    my = resp.yData


    mag_msg = MagneticField(
        header=Header(
            stamp=rospy.Time.now(),
            frame_id='/map',
        ),
        magnetic_field=Vector3(x=mx, y=my, z=mz), 
        magnetic_field_covariance=
            [0.03**2, 0,       0,
             0,       0.03**2, 0,
             0,       0,       0.03**2,],
    )
    print mag_msg
    mag_pub.publish(mag_msg)