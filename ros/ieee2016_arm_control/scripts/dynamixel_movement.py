#using dynamixel_io file
#moving some dynamixels nd shit
#29000 - 36000 dead zone
import rospy
from dynamixel_io import DynamixelIO

d = DynamixelIO('/dev/ttyUSB3', 1000000)

#given a distance, move to the maximum distance
#need 12 rotations
#get initial position
start_position = d.get_position(2)
print start_position
#set initial to 28000
if start_position != 28000:
	d.set_position(2, 28000)
#checker of positions
while d.get_position(2) != 28000:
	print d.get_position(2)
print "finished"
#turn to final position of 37000 for max extenstion
final_position = 37000
print final_position
d.set_position(2, final_position)
# d.set_position(2, 0) for position reset