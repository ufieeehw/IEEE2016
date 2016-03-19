import maestro
import cv2
import numpy as np
import threading
import os
import yaml

CALIBRATION_FILE = "calibration.yaml"

class CalibrateServos():
    '''
    A calibrator for the servos.

    This calibrator uses port numbers to select the servo, then to adjust values use the
    opencv trackbar. Currently only supports 8 servos

    Commands:
            'X': sets the current servo (where X is an integer).
            'closed': sets the current position as the closed position for the current servo.
            'opened': sets the current position as the open position for the current servo.
            'close': moves servo to closed position.
            'open': moves servo to opened position.
            'load': loads past calibration data. Loading overwrites current calbriation data.
            'save': saves the calibration data. Saving overwrites past calibration data.
            'exit_save': exits and saves calibration data. Saving overwrites past calibration data. 
            'exit': exits without saving calibration data.
            'help': lists this help information."

    Due to some threading stuff, ctrl-C maybe need to be followed by the return key.
    '''
    def __init__(self, min_range=672, max_range=2608):
        self.running = True
        img = np.zeros((1,1000), np.uint8)
        cv2.namedWindow('parameters')
        cv2.createTrackbar('value','parameters', min_range, max_range, self.update_servo_position)

        self.min_range = min_range
        self.max_range = max_range

        self.servos = maestro.Controller()
        self.current_servo = None
        self.current_position = min_range
        self.load_data()

        command_listener = threading.Thread(target=self.command_listener)
        command_listener.start()

        for servo in range(12):
            self.servos.set_range(servo, self.min_range, self.max_range)
            self.servos.set_speed(servo, 200)
            self.servos.set_acceleration(servo, 200)

        while self.running:
            try:
                cv2.imshow('parameters',img)
                cv2.waitKey(1)
            except KeyboardInterrupt:
                # Listen for control C
                self.exit(False)

    def update_servo_position(self, position):
        if self.current_servo is not None:
            if position < self.min_range or position > self.max_range:
                return
            self.current_position = position
            self.servos.set_target(self.current_servo, self.current_position)
            #print position

    def command_listener(self):
        while self.running:
            command = raw_input(" > ")
            # We are going to look through each command in the commands dicationary. If the entry isnt there, assume
            # the user is trying to change servos.
            if command == 'closed':
                this_data = self.calibration_data[self.current_servo]
                this_data['closed'] = self.current_position
                print "Saved %i as servo %i's closed position."%(self.current_position,self.current_servo)

            elif command == 'opened':   
                this_data = self.calibration_data[self.current_servo]
                this_data['opened'] = self.current_position
                print "Saved %i as servo %i's opened position."%(self.current_position,self.current_servo)
            
            elif command == 'close':
                closed_position = self.calibration_data[self.current_servo]['closed']
                self.servos.set_target(self.current_servo, closed_position)
            
            elif command == 'open':
                opened_position = self.calibration_data[self.current_servo]['opened']
                self.servos.set_target(self.current_servo, opened_position)

            elif command == 'load':
                self.load_data()
            
            elif command == 'save':
                self.save_data()

            elif command == 'exit_save':
                self.exit(True)

            elif command == 'exit':
                self.exit(False)

            elif command == 'help':
                print "Current implemented commands:\n",\
                "'X': sets the current servo (where X is an integer).\n",\
                "'closed': sets the current position as the closed position for the current servo.\n",\
                "'opened': sets the current position as the open position for the current servo.\n",\
                "'load': loads past calibration data. Loading overwrites current calbriation data.\n",\
                "'save': saves the calibration data. Saving overwrites past calibration data.\n",\
                "'exit_save': exits and saves calibration data. Saving overwrites past calibration data. \n",\
                "'exit': exits without saving calibration data.\n",\
                "'help': lists this help information."
            else:
                try:
                    self.current_servo = int(command)
                    print "Switching to servo:",self.current_servo

                except:
                    print "Error, not a valid command. Type 'help' for more info."
                    continue

    def load_data(self):
        '''
        Calibration data is formated as a dictionary:
        {servo_number:{closed, opened}, ...}
        '''
        with open(CALIBRATION_FILE, 'r') as infile:
            self.calibration_data = yaml.load(infile)
        print "Data Loaded."

    def save_data(self):
        with open(CALIBRATION_FILE, 'w') as outfile:
            outfile.write(yaml.dump(self.calibration_data, default_flow_style=False))
        print "Data Saved."

    def exit(self, save):
        if save:
            self.save_data()
            print "Closing Calibrator and Saving."
            cv2.destroyAllWindows()
            self.running = False
        else:
            print "Closing Calibrator and NOT Saving."
            cv2.destroyAllWindows()
            self.running = False

if __name__ == "__main__":
    c = CalibrateServos()