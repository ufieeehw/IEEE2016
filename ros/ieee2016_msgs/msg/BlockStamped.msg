# 2D Point in a camera frame representing a block on the map.

std_msgs/Header header		# Stamped with the *CAMERA NAME* and the time image was taken.
int32[2] point			# The UV coordinate of the block.
string color			# Color of the block.

