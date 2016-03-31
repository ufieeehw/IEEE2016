#using dynamixel_io file
#moving some dynamixels nd shit
#29000 - 36000 dead zone
import rospy
from dynamixel_io import DynamixelIO

class DynamixelController():
	def __init__(self):
		#self.d = DynamixelIO('/dev/ttyUSB3', 1000000)
		self.position = []
		#self.position[0] = self.d.get_position(1)
		#self.position[1] = self.d.get_position(2)
		#self.position[2] = self.d.get_position(3)
		self.position.append(3)
		self.position.append(2)
		self.position.append(1)

	#function that gets position, set as zero point, set multi, pass index & desired position & radius (m)
	def set_dynamixels(self, servo_ids, position, radius):
		servo_id = []
		for index in servo_ids:
			#position in ticky tick ticks
			final_position = position/(0.088 * ((2*3.1415926535)/360) * radius)
			print final_position
			final_position = self.position[index - 1] - final_position
			#adjusting for negatives
			if final_position < 0:
				final_position = 65536 + final_position
			servo_id.append((index, int(round(final_position))))
		print servo_id
		#d.set_multi_position((1, final_position), (2, final_position), (3, final_position))

	#state of where it's at, how far it rotated
	def dynamixel_state(self, servo_ids, radius):
		servo_id = []
		for index in servo_ids:
			#started from the bottom
			start_position = self.position[index - 1]
			#now we here
			final_position = 30
			print start_position
			#how far did we go tho
			distance_rotated = start_position - final_position
			if distance_rotated > 28000:
				final_position = 65536 - self.position[index - 1] + start_position
				distance_rotated = start_position - final_position
			if distance_rotated < 0:
				start_position = 65536 + start_position
				distance_rotated = start_position - final_position
			servo_id.append((index, int(round(distance_rotated))))
			#position in meepy meters
			distance_meters = distance_rotated * (0.088 * ((2*3.1415926535)/360))
		print servo_id
		print distance_meters

d = DynamixelController()
d.set_dynamixels([1, 2], .006, 1)
d.dynamixel_state([1, 2], 1)