#!/usr/bin/env python
import numpy as np
import cv2
import math
import rospy
import tf

from sensor_msgs.msg import LaserScan
import std_msgs
import time

def nothing(x):
    pass

#should be the right scanner
def logData(data,logID):
    global ranges,logging
    if logID == 0:
        ranges[0] = returnRange(125,data)
        logging[0] = True  
        #print "ranges0 updated"
    if logID == 1:
        ranges[1] = returnRange(0,data)
        logging[1] = True 
        #print "ranges1 updated"
    if logID == 2:
        ranges[2] = returnRange(125,data)
        logging[2] = True 
        #print "ranges2 updated"        


#displays x y point on graph
def convertxypoint(x,y):
    global h,w
    u = h/2.0 + x
    v = w/2.0 - y

    #cv2.circle(img,(xMod,yMod),1,0)
    return u,v

def rosPublish(ranges,maxAngle,minAngle,increment):
    global pub
    #set up ros message
    h = std_msgs.msg.Header()
    h.frame_id = "laser_comb"

    l = LaserScan()
    l.angle_min = minAngle
    l.angle_max = maxAngle
    l.angle_increment = increment
    l.time_increment = 0.0
    l.scan_time = 0.0
    l.range_min = 0.01
    l.range_max = 5.0
    l.ranges = ranges
    l.intensities = []

    #update time info to keep transforms right
    h.stamp = rospy.Time.now()
    l.header = h

    #print "Publishing..."
    pub.publish(l)

def returnAngle(x,y):
    theta = math.atan(y/x)
    if x >= 0 and y >= 0:
        return theta
    if x <= 0 and y >= 0:
        return theta + math.pi
    if x <= 0 and y <= 0:
        return theta - math.pi
    if x >= 0 and y <=0:
        return theta

#converts cartesian 'carts' back to the laserscan data and publishes it
def convertBackToArray(carts):
    x = carts[0]
    y = carts[1]
    dataPoints = len(x)

    #define max and min angles
    maxAngle = math.pi
    minAngle = -math.pi
    increment = 0.003 #this is the target increment for the output

    #create list of lists to be used as angle compartments
    #the elements of each compartment will be averaged
    compRanges = [[]]#np.zeros(int((abs(maxAngle)+abs(minAngle))/increment)+2)
    for i in range(int((abs(maxAngle)+abs(minAngle))/increment)):
        compRanges.append([])

    for i in range(len(x)):
        r = math.sqrt(math.pow(x[i],2)+math.pow(y[i],2))/100.0
        angle = returnAngle(x[i],y[i])

        #find the 'compartment' where the point belongs
        compartment = int((angle-minAngle)/increment)
        #print dataPoints,compartment
        compRanges[compartment].append(r)

        #just for displaying
        #tempu,tempv = convertxypoint(x[i],y[i])
        #cv2.circle(img,(int(tempu),int(tempv)),1,(0,0,0),-1)
        #cv2.imshow('img',img)
        #cv2.waitKey(1)


    #average distances in each compartment
    for i,c in enumerate(compRanges):
        arrSum = 0
        for val in c:
            arrSum += val
        if len(c) > 0:
            compRanges[i] = arrSum/float(len(c))
        else:
            compRanges[i] = float('inf')

    compRanges[0] = float('inf')
    compRanges[len(compRanges)-1] = float('inf')
    rosPublish(compRanges,maxAngle,minAngle,increment)

#coverts 'ranges' to cartesian with 'trans' tf data applied to it
def convertToCart(ranges,trans,scanNum):
    global angle_increment

    dataPoints = len(ranges)
    if dataPoints == 0:
        return
    #create a temporary array to hold transformed points
    temp = np.zeros(shape=(2,dataPoints))
    
    #decompile trans data, only dx,dy,theta will be used

    q = tf.transformations.quaternion_from_euler(1.570796, 0, 3.14159)
    #print trans[0],tf.transformations.euler_from_quaternion(trans[1])
    #dx,dy,dz,theta,ry,rz = trans
    dx,dy,dz = trans[0]
    # No clue why theta is given as a negative number here, but this function gives it as negative
    rx,ry,theta = tf.transformations.euler_from_quaternion(trans[1])
    theta = theta
    print dx,dy,dz,rx,ry,theta
    
    #print "Starting conversion"
    for i in range(dataPoints):
        #convert polar to angle and distance
        angle = -dataPoints*angle_increment/2.0 + i*angle_increment 
        dist = ranges[i]*100.0 #convert to cm
        #sometimes dist values are too far and it'll return inf or nan but we don't want that
        if dist > .001 and dist != float('inf'): 
            #add converted values to temp arrays and flip the y values since the LIDARS are upsidedown
            temp[0,i] = math.cos(angle)*dist
            temp[1,i] = -math.sin(angle)*dist
        else:
            #so the angles stay right, fill with a big big value
            temp[0,i] = 0
            temp[1,i] = 0

    #create a rotation matrix to transform temp based on provided tf theta
    rotMat = np.matrix([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    cartMod = rotMat*temp
    
    #transform points in the x,y plane based on provided tf values
    cartMod[0] += dx*100
    cartMod[1] += dy*100

    #this is only for displaying points
    u,v = [], []
    for i in range(dataPoints):
        utemp,vtemp = convertxypoint(cartMod[0,i],cartMod[1,i])
        u.append(utemp)
        v.append(vtemp)

    if scanNum == 0:
        for i in range(dataPoints):
            cv2.circle(img,(int(u[i]),int(v[i])),2,(0,0,255),-1)
    if scanNum == 1:
        for i in range(dataPoints):
            cv2.circle(img,(int(u[i]),int(v[i])),2,(0,255,0),-1)
    if scanNum == 2:
        for i in range(dataPoints):
            cv2.circle(img,(int(u[i]),int(v[i])),2,(255,0,0),-1)

    return cartMod#[::-1]


#returns indicies for a 'deg' angle centered in the data ranges
def returnRange(deg,data):
    angle_increment = data.angle_increment
    ranges = data.ranges
    midPoint =  len(ranges)/2.0
    rads = math.radians(deg)

    maxIndex = int(midPoint + rads/(2*angle_increment))
    minIndex = int(midPoint - rads/(2*angle_increment))

    tempLength = maxIndex-minIndex
    temp = np.zeros(tempLength)
    temp = ranges[minIndex:maxIndex]
    return temp


# ~main function~

#define parameters increasing these increases search size
h = 450
w = h

#ros stuffs
pub = rospy.Publisher('scan_comb', LaserScan, queue_size=3)
rospy.init_node('laser_comb')
rospy.Subscriber('scan_left',LaserScan,logData,0)
rospy.Subscriber('scan_middle',LaserScan,logData,1) 
rospy.Subscriber('scan_right',LaserScan,logData,2) 
r = rospy.Rate(10) # 10hz

#Hardcoded LIDAR data
angle_min = -1.57079637051
angle_max = 1.56466042995
angle_increment = (angle_max-angle_min)/512
print angle_increment
#make white background
img = np.zeros((h,w,3), np.uint8)+255

#blank lists to hold data from ros topics
ranges = [False,False,False]
logging = [False,False,False]

#will be replaced with tf data
listener = tf.TransformListener()
#tTemp = (0 ,0, 0, -math.pi, 0, 3.141592) 
#t0 = (0 ,-.125, 0, -1.570796, 0, 3.141592) 
#t1 = (.1, 0, 0, 0, 0, 3.141592) 
#t2 = (0, .125, 0, 1.570796, 0, 3.141592) 
cv2.waitKey(2000)
while not rospy.is_shutdown():
    startTime = time.time()

    #clear display
    img = np.zeros((h,w,3), np.uint8)+255

    #draw axis
    cv2.line(img,(0,h/2),(w,h/2),(0,0,0),2)
    cv2.line(img,(w/2,0),(w/2,h),(0,0,0),2)

    #create a blank array to hold the composed cartesian coordinates
    cartComp = np.array([np.empty(0),np.empty(0)])
    print logging
    if logging[0]:
        try:
            print "Logging 0"
            t0 = listener.lookupTransform('/base_link', '/laser_left', rospy.Time(0))
            temp = convertToCart(ranges[0],t0,0)
            cartComp = np.append(cartComp[0],temp[0]),np.append(cartComp[1],temp[1])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            logging[0] = False
            continue     
    if logging[1]:
        try:
            print "Logging 1"
            t1 = listener.lookupTransform('/base_link', '/laser_middle', rospy.Time(0))
            temp = convertToCart(ranges[1],t1,1)
            cartComp = np.append(cartComp[0],999),np.append(cartComp[1],999)#cartComp = np.append(cartComp[0],temp[0]),np.append(cartComp[1],temp[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            logging[1] = False
            continue   
    if logging[2]:
        try:
            print "Logging 2"
            t2 = listener.lookupTransform('/base_link', '/laser_right', rospy.Time(0))
            temp = convertToCart(ranges[2],t2,2)
            cartComp = np.append(cartComp[0],temp[0]),np.append(cartComp[1],temp[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            logging[2] = False
            continue   

    #display everything
    cv2.imshow('img',img)


    #print cartComp[0]
    if logging[0] or logging[1] or logging[2]:
        #print "Converting"
        #s.unregister()
        convertBackToArray(cartComp)    

    print "Hz:",1.0/(time.time()-startTime)

cv2.destroyAllWindows()