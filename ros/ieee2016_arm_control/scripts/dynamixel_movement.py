#using dynamixel_io file
#moving some dynamixels nd shit
#29000 - 36000 dead zone
import rospy
from dynamixel_io import DynamixelIO
import time

RADS_PER_TICK = 0.00153588974

class DynamixelController():
    def __init__(self):
        # This checks for the correct port to use. That way we don't have to do any junk with udev.
        real_port = None
        for port_index in range(50):
            port = '/dev/ttyUSB%d'%port_index
            try:
                self.d = DynamixelIO(port, 1000000)
                if len(self.d.ping(1)) > 1 or len(self.d.ping(2)) > 1 or len(self.d.ping(3)) > 1:
                    real_port = port
                    print "USING PORT %s!"%port
                    break
                print "PORT %s NOT CORRECT."%port
            except:
                print "PORT %s NOT AVAILABLE."%port
                pass

        if real_port is None:
            print "NO PORT FOUND!"
            exit()

        self.position = []
        # All the trys are just for testing.
        try:
            self.position.append(self.d.get_position(1))
            #self.d.set_torque_enabled(1,0)
        except:
            self.position.append(None)
        try:
            self.position.append(self.d.get_position(2))
            #self.d.set_torque_enabled(2,0)
        except:
            self.position.append(None)
        try:
            self.position.append(self.d.get_position(3))
            #self.d.set_torque_enabled(3,0)
        except:
            self.position.append(None)
    

    def reset_inital_position(self, servo_id):
        self.position[servo_id - 1] = self.d.get_position(servo_id)

    #function that gets position, set as zero point, set multi, pass index & desired position & radius (m)
    def set_dynamixels(self, servo_ids, position, radius):
        servo_id = []
        for index in servo_ids:
            #position in ticky tick ticks. 5.042028597 rads / tick = .088 degrees / tick
            final_position = (position / radius) / (RADS_PER_TICK)
            print final_position
            final_position = self.position[index - 1] - final_position
            #adjusting for negatives
            if final_position < 0:
            	final_position = 65536 + final_position
            if final_position > 65536:
                final_position = final_position - 65536
            servo_id.append(( index, int(round(final_position)) ))
        print "SETTING:", servo_id
        self.d.set_multi_position(servo_id)

    #state of where it's at, how far it rotated
    def dynamixel_state(self, servo_id, radius):
        # ********* Not sure if this returns accurate data. *********
        #started from the bottom
        start_position = self.position[servo_id - 1]
        #now we here
        final_position = self.d.get_position(servo_id)
        print start_position
        print final_position
        #how far did we go tho
        distance_rotated = final_position - start_position
        print distance_rotated
        if distance_rotated < 0:
        	start_position = 65536 + start_position
        	distance_rotated = final_position - start_position

        #position in meepy meters
        distance_meters = distance_rotated * RADS_PER_TICK / radius

        print "SERVO:",servo_id
        print "DISTANCE:",distance_meters
        #return distance_meters

    def dynamixel_motion(self, servo_id):
        print self.d.get_speed(servo_id)
    	if self.d.get_speed(servo_id) > 10:
            print "True"
            return True
        else:
            print "False"
            return False

if __name__ == "__main__":
    temp = DynamixelController()
    temp.set_dynamixels([3], -.1, .015875)
    #temp.set_dynamixels([1,2], -.1, .00808507)
    time.sleep(1)
    print
    temp.dynamixel_state(3, .00808507)
