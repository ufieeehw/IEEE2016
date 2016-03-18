# 2D Point in a camera frame representing a block on the map.

std_msgs/Header header		# Stamped with the *CAMERA NAME* and the time image was taken.
int32[2] point			# The UV coordinate of the block.
float32 offset			# Offset distance past the wall in meters (the surface of the wall is 0,
				# 	           negative is infront of the wall, positive is behind.)
string color			# Color of the block.

