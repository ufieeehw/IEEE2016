#!/usr/bin/python2
#=============================================================================
# Project: IEEE 2016 Hardware Team Robot (Shia LaBot)
# Module: Color Calibrator												v1.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

from PyQt4 import QtCore, QtGui
import sys
import threading
import time

import camera_manager
from color_calibration import Calibrations
from color_calibrator_gui import Ui_MainWindow
from color_detection import Image, ObjectDetection
import rospy


try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class OutputWrapper(QtCore.QObject):
	'''
	The class used as an output wrapper for messages originating at stdout and
	stderr.
	'''
	outputWritten = QtCore.pyqtSignal(object, object)

	def __init__(self, parent, stdout = True):
		QtCore.QObject.__init__(self, parent)
		if stdout:
			self._stream = sys.stdout
			sys.stdout = self
		else:
			self._stream = sys.stderr
			sys.stderr = self
		self._stdout = stdout

	def write(self, text):
		'''
		This method is called to print the output text.
		'''
		self._stream.write(text)
		self.outputWritten.emit(text, self._stdout)  # Signal to initiate the text box action

	def __getattr__(self, name):
		return getattr(self._stream, name)

	def __del__(self):
		try:
			if self._stdout:
				sys.stdout = self._stream
			else:
				sys.stderr = self._stream
		except AttributeError:
			pass


class ColorCalibrator(QtGui.QMainWindow, Ui_MainWindow):
	'''
	This class contains all of the functions that operate to change the user
	interface and change variables based on user interface events.
	'''
	def __init__(self, calibration):
		self.calibration = calibration
		self.display = ImageDisplay(self)

		self.selection_color = ""
		self.detection_color = ""
		self.display_frame = "unfiltered"
		self.averaging = 2
		self.colors = {}
		self.prevention_setting = {}

		super(self.__class__, self).__init__()
		self.setupUi(self)
		self.connect_ui()
		self.update_color_list()
		self.show()

	def connect_ui(self):
		'''
		Connects signals on the UI objects to actual backend code in order to
		facilitate running the three subsystems.
		'''
		# Connect frame updates from the ImageDisplay to the gui update method
		self.display.selection_frame_updated.connect(lambda: self.update_frame(self.display.selection_frame, "selection"))
		self.display.detection_frame_updated.connect(lambda: self.update_frame(self.display.detection_frame, "detection"))

		# Connect the color setting functions to all relevant methods
		self.selection_color_setting.activated.connect(self.update_selection_color)
		self.detection_color_setting.activated.connect(self.update_detection_color)

		# Connect the calibration management buttons to the calibration object
		self.delete_color_button.clicked.connect(self.delete_calibration)

		# Link the point control sliders with their roll boxes
		self.connect(self.x_coordinate_setting, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x_coordinate_slider.setValue)
		self.connect(self.x_coordinate_slider, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x_coordinate_setting.setValue)
		self.connect(self.y_coordinate_setting, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y_coordinate_slider.setValue)
		self.connect(self.y_coordinate_slider, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y_coordinate_setting.setValue)

		# Manages the values for the X and Y coordinate sliders and the averaging slider
  		self.x_coordinate_slider.valueChanged[int].connect(self.update_x_coordinate)
		self.y_coordinate_slider.valueChanged[int].connect(self.update_y_coordinate)
		self.averaging_setting.valueChanged[int].connect(self.update_averaging)

		# Link the display type selectors to the display_frame variable
		self.display_unfiltered_button.clicked.connect(self.change_detection_display)
		self.display_reduced_button.clicked.connect(self.change_detection_display)
		self.display_extracted_button.clicked.connect(self.change_detection_display)

		# Link the HSV display buttons to their display update functions
		self.save_changes_button.clicked.connect(self.save_hsv_boxes)
		self.undo_changes_button.clicked.connect(self.load_hsv_boxes)
		self.minimum_hsv_text.returnPressed.connect(self.save_hsv_boxes)
		self.maximum_hsv_text.returnPressed.connect(self.save_hsv_boxes)

		# Redirect the output of stdout and stderr to an output wrapper class
		stdout = OutputWrapper(self, True)
		stdout.outputWritten.connect(self.printOutput)
		stderr = OutputWrapper(self, False)
		stderr.outputWritten.connect(self.printOutput)

	def update_frame(self, frame, display_to):
		'''
		Updates the frame displayed in one of the two display areas.
		'''
		# Reformats the image as a pixmap
		image = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
		pixmap = QtGui.QPixmap.fromImage(image)

		# Embeds the pixmap into the selected label object
		if (display_to == "selection"):
			self.selection_frame.setPixmap(pixmap)
		if (display_to == "detection"):
			self.detection_frame.setPixmap(pixmap)

	def update_color_list(self):
		'''
		Runs when the available colors are changed. Updates both of the color
		setting boxes and the overlap prevention list.
		'''
		item = 0

		# Clears the existing items in the relevant objects
		self.selection_color_setting.clear()
		self.detection_color_setting.clear()
		self.colors = {}
		for setting in self.prevention_setting:
			self.prevention_setting[setting].setParent(None)
		self.prevention_setting = {}

		for color in self.calibration.available_colors:
			# Adds an entry to both of the color setting boxes for each color
			self.selection_color_setting.addItem(_fromUtf8(""))
			self.selection_color_setting.setItemText(item, _translate("MainWindow", color, None))
			self.detection_color_setting.addItem(_fromUtf8(""))
			self.detection_color_setting.setItemText(item, _translate("MainWindow", color, None))

			# Adds an entry to the overlap prevention scroll area for each color
			self.prevention_setting[item] = QtGui.QCheckBox(self.overlap_prevention_colors)
			self.prevention_setting[item].setObjectName(_fromUtf8("%s_prevention_setting" % (color)))
			self.verticalLayout_8.addWidget(self.prevention_setting[item])
			self.prevention_setting[item].setText(_translate("MainWindow", color, None))

			# A dictionary storing which index each color is stored under
			self.colors[item] = color
			item += 1

		# Selects previously set selection color from index if available
		for index in range(len(self.colors)):
			if (self.colors[index] == self.selection_color):
				self.selection_color_setting.setCurrentIndex(index)
				break
			else:
				self.selection_color_setting.setCurrentIndex(-1)
		self.update_selection_color()

		# Selects previously set detection color from index if available
		for index in range(len(self.colors)):
			if (self.colors[index] == self.detection_color):
				self.detection_color_setting.setCurrentIndex(index)
				break
			else:
				self.detection_color_setting.setCurrentIndex(-1)
		self.update_detection_color()

	def update_selection_color(self):
		'''
		Updates the selection_color variable and manages the selection frame
		display thread.
		'''
		if (self.colors and not (self.selection_color_setting.currentIndex() < 0)):

			# Set the selection color
			self.selection_color = self.colors[self.selection_color_setting.currentIndex()]

			# Attempt to start frame display if it is not already running
			try:
				if (self.display.loop_selection == False):
					self.display.loop_selection = True
					self.selection_thread = threading.Thread(target = self.display.get_selection_frame)
					self.selection_thread.daemon = True
					self.selection_thread.start()
			except:
				pass

		# If no selection color is set, disable frame display
		else:
			self.selection_color = ""
			try:
				self.display.loop_selection = False
			except:
				pass

	def update_detection_color(self):
		'''
		Updates the the detection_color variable, manages the detection frame
		display thread, and manages the color HSV ranges display and editor based on
		the set detection color.
		'''
		# Displays the proper ranges for the set color
		if (self.colors and not (self.detection_color_setting.currentIndex() < 0)):

			# Set the detection color
			self.detection_color = self.colors[self.detection_color_setting.currentIndex()]

			# Attempt to start frame display if it is not already running
			try:
				if (self.display.loop_detection == False):
					self.display.loop_detection = True
					self.detection_thread = threading.Thread(target = self.display.get_detection_frame)
					self.detection_thread.daemon = True
					self.detection_thread.start()
			except:
				pass

			# Enable the HSV value display and populate it
			self.minimum_hsv_text.setEnabled(True)
			self.maximum_hsv_text.setEnabled(True)
			self.load_hsv_boxes()

		# Disables the display and editor if no color is set
		else:
			self.detection_color = ""

			# Attempt to start
			try:
				self.display.loop_detection = False
			except:
				pass

			# Clear and disable the HSV value display
			self.minimum_hsv_text.clear()
			self.maximum_hsv_text.clear()
			self.minimum_hsv_text.setEnabled(False)
			self.maximum_hsv_text.setEnabled(False)

	def delete_calibration(self):
		'''
		Removes a calibration from memory only. Also updates the list of
		available colors on the user interface.
		'''
		if not (self.selection_color == ""):
			self.calibration.delete(self.colors[self.selection_color_setting.currentIndex()])
			self.update_color_list()
		else:
			print("ERROR: Cannot delete a nonexistant calibration")

	def update_x_coordinate(self, value):
		'''
		INCOMPLETE: Sets the x coordinate value based on it's slider
		'''
		self.x_coordinate = value

	def update_y_coordinate(self, value):
		'''
		INCOMPLETE: Sets the y coordinate value based on it's slider
		'''
		self.y_coordinate = value

	def update_averaging(self, value):
		'''
		Update the averaging value based on it's spin box's setting.
		'''
		self.averaging = value

	def change_detection_display(self):
		'''
		Sets the frame to display on the detection side of the GUI.
		'''
		if self.display_unfiltered_button.isChecked():
			self.display_frame = "unfiltered"
		if self.display_reduced_button.isChecked():
			self.display_frame = "reduced"
		if self.display_extracted_button.isChecked():
			self.display_frame = "extracted"

	def printOutput(self, text):
		'''
		Used to print the output of stdout and stderr to a text box.
		'''
		self.log_output_box.moveCursor(QtGui.QTextCursor.End)
		self.log_output_box.insertPlainText(text)

	def load_hsv_boxes(self):
		'''
		Prints the maximum and minimum HSV ranges for the set detection color
		to their respective text boxes. If no color is selected, clears the
		box. Also used to reset the ranges if changes are undone by the user.
		'''
		if (self.detection_color):
			minimum_values = self.calibration.calibrations[self.detection_color][0]
			maximum_values = self.calibration.calibrations[self.detection_color][1]
			self.minimum_hsv_text.setText("(%d, %d, %d)" % (minimum_values[0], minimum_values[1], minimum_values[2]))
			self.maximum_hsv_text.setText("(%d, %d, %d)" % (maximum_values[0], maximum_values[1], maximum_values[2]))
		else:
			self.minimum_hsv_text.clear()
			self.maximum_hsv_text.clear()

	def save_hsv_boxes(self):
		'''
		Saves the HSV ranges currently in the boxes to the calibrations object
		if they are properly formatted tupples within the acceptable range of
		[0, 255]. NOTE: This does not save the values to the file!
		'''
		minimum = self.minimum_hsv_text.text()
		maximum = self.maximum_hsv_text.text()

		# Determine if the tupple is properly formatted to (x, y, z)
		if not (maximum and minimum):
			print("ERROR: One of the HSV fields is blank")
			return None
		elif (minimum[0] == '(' and maximum[0] == '(' and minimum[len(minimum) - 1] == ')' and maximum[len(maximum) - 1] == ')'):
			minimum_values = tuple(int(char) for char in minimum[1:-1].split(','))
			maximum_values = tuple(int(char) for char in maximum[1:-1].split(','))
		else:
			print("ERROR: One of the HSV ranges is not formatted properly")
			return None

		# Ensures that the HSV values are within the range limits
		for value in minimum_values:
			if not (value >= 0 and value <= 255):
				print("ERROR: One of the HSV values is out of the expected range of [0, 255]")
				return None
		for value in maximum_values:
			if not (value >= 0 and value <= 255):
				print("ERROR: One of the HSV values is out of the expected range of [0, 255]")
				return None

		# Saves the values to the calibration object
		self.calibration.calibrations[self.detection_color][0] = minimum_values
		self.calibration.calibrations[self.detection_color][1] = maximum_values

		# Updates the numpy calibration values to reflect the changes for cv2
		self.calibration.load_cv2()

	def closeEvent(self, event):
		'''
		Used to ensure clean termination of all threads when the main window is
		closed.
		'''
		self.display.loop_selection = False
		self.display.loop_detection = False


class ImageDisplay(QtCore.QObject):
	'''
	Holds the threads used to generate and display the images necessary
	requested by the GUI.
	'''
	selection_frame_updated = QtCore.pyqtSignal()
	detection_frame_updated = QtCore.pyqtSignal()

	def __init__(self, gui):
		QtCore.QObject.__init__(self)
		self.gui = gui
		self.camera = camera

		self.loop_selection = False
		self.loop_detection = False
		self.selection_frame = None
		self.detection_frame = None

	def get_selection_frame(self):
		'''
		INCOMPLETE: Currently prints the resized frame from the image object.
		'''
		# Separate image manipulation object for this frame
		image = Image(camera, calibration, 320)

		while (self.loop_selection == True):
			image.resize()
			image.reformat_to_rgb()
			self.selection_frame = image.frame
			self.selection_frame_updated.emit()
			time.sleep(0.017)

	def get_detection_frame(self):
		'''
		Detects the set detection color and draws a bounding box and center
		point for it on the requested frame type.
		'''
		# Separate image manipulation and detection objects for this frame
		image = Image(camera, calibration, 320)
		detect = ObjectDetection(self.camera, calibration, image)

		while (self.loop_detection == True):
			color = self.gui.detection_color
			averaging = self.gui.averaging

			# Average detection of the selected color to obtain box and center points
			detect.average_box(detect.select_largest_object, [color], averaging)
			detect.get_box_center()

			# Pull the requested frame to display the image on
			if (self.gui.display_frame == "unfiltered"):
				image.resize()
			elif (self.gui.display_frame == "reduced"):
				image.reduce_colors(16)
			elif (self.gui.display_frame == "extracted"):
				image.extract_color(self.gui.detection_color)

			# Draw the detection bounding box and center point if one exists
			detect.draw_box([color], (255, 255, 255))
			detect.draw_center_point([color], (255, 255, 255))

			# Reformat the image to RGB, which is what QImage takes, and emit an update signal
			image.reformat_to_rgb()
			self.detection_frame = image.frame
			self.detection_frame_updated.emit()

			# Keep the refresh rate at or below 60 FPS
			time.sleep(0.017)


if __name__ == '__main__':
	# ROS, color calibration, and camera dependencies
	rospy.init_node("color_calibrator")
	camera = camera_manager.Camera(1)
	camera.activate()
	calibration = Calibrations()

	# Launching the application
	app = QtGui.QApplication(sys.argv)
	gui = ColorCalibrator(calibration)
	sys.exit(app.exec_())
	camera.deactivate()
