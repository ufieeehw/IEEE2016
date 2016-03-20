#!/usr/bin/python2
#=============================================================================
# Project: Machine Vision - Color Detection
# Module: Color Calibrator												v1.7
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

from PyQt4 import QtCore, QtGui
import os
import sys
import threading
import time

from camera_stream import Camera
from color_calibration import CalibrationData
from color_calibrator_gui import Ui_MainWindow
from color_detection import Image, ObjectDetection

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


class ColorCalibrator(QtGui.QMainWindow, Ui_MainWindow):
	'''
	This class contains all of the functions that operate to change the user
	interface and change variables based on user interface events.
	'''
	def __init__(self):
		self.display = ImageDisplay(self)

		# Variables used only for running the backend code
		self.colors = {}
		self.new_msg_printed = False

		# Parallel threads for image display
		self.selection_thread = threading.Thread(target = self.display.get_selection_frame)
		self.detection_thread = threading.Thread(target = self.display.get_detection_frame)

		# Variables that can be modified in the GUI
		self.selection_color = ""
		self.detection_color = ""
		self.display_frame = "unfiltered"
		self.averaging = 2
		self.prevention_setting = {}

		super(self.__class__, self).__init__()
		self.setupUi(self)
		self.connect_ui()
		self.no_file_interface()
		self.show()

	def connect_ui(self):
		'''
		Connects signals on the UI objects to actual backend code in order to
		facilitate running the three subsystems.
		'''
		# Enable menu file manipulation and program quitting
		self.menu_new.triggered.connect(lambda: self.load_file("new"))
		self.menu_save.triggered.connect(self.save_file)
		self.menu_load.triggered.connect(lambda: self.load_file("load"))
		self.menu_quit.triggered.connect(self.closeEvent)

		# Connect frame updates from the ImageDisplay to the gui update method
		self.display.selection_frame_updated.connect(lambda: self.update_frame(self.display.selection_frame, "selection"))
		self.display.detection_frame_updated.connect(lambda: self.update_frame(self.display.detection_frame, "detection"))

		# Connect the color setting functions to all relevant methods
		self.selection_color_setting.activated.connect(self.update_selection_color)
		self.detection_color_setting.activated.connect(self.update_detection_color)

		# Connect the calibration management buttons to the calibration object
		self.new_color_name_setting.cursorPositionChanged.connect(self.clear_new_name)
		self.new_color_name_setting.editingFinished.connect(self.print_new_name_msg)
		self.new_color_name_setting.returnPressed.connect(self.create_new_calibration)
		self.new_color_button.clicked.connect(self.create_new_calibration)
		self.delete_color_button.clicked.connect(self.delete_calibration)

		# Manages the values for the X and Y coordinate sliders and the averaging slider
		self.x1_slider.valueChanged[int].connect(self.update_x1_coordinate)
		self.y1_slider.valueChanged[int].connect(self.update_y1_coordinate)
		self.x2_slider.valueChanged[int].connect(self.update_x2_coordinate)
		self.y2_slider.valueChanged[int].connect(self.update_y2_coordinate)
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

	def no_file_interface(self):
		'''
		Disables all gui objects that are not useable until a file is loaded.
		'''
		# Menu
		self.menu_save.setEnabled(False)

		# Color selection
		self.selection_color_label.setEnabled(False)
		self.selection_color_setting.setEnabled(False)

		# Color management
		self.new_color_name_setting.setEnabled(False)
		self.new_color_button.setEnabled(False)
		self.delete_color_button.setEnabled(False)

		# Selection management
		self.selection_managment_group.setEnabled(False)

		# Overlap prevention
		self.overlap_prevention_group.setEnabled(False)

		# Calibration button
		self.detection_color_label.setEnabled(False)
		self.calibrate_button.setEnabled(False)

		# Detection management
		self.detection_color_label.setEnabled(False)
		self.detection_color_setting.setEnabled(False)

		# Color detection
		self.selections_to_average_label.setEnabled(False)
		self.averaging_setting.setEnabled(False)

		# Display type
		self.display_type_group.setEnabled(False)

		# Color HSV Ranges
		self.color_ranges_group.setEnabled(False)

	def file_loaded_interface(self):
		'''
		Enables all gui objects that are not dependent on colors being set.
		'''
		# Menu
		self.menu_save.setEnabled(True)

		# Color selection
		self.selection_color_label.setEnabled(True)
		self.selection_color_setting.setEnabled(True)

		# Color management
		self.new_color_name_setting.setEnabled(True)
		self.new_color_button.setEnabled(True)
		self.delete_color_button.setEnabled(True)

		# Color detection
		self.selections_to_average_label.setEnabled(True)
		self.averaging_setting.setEnabled(True)

		# Detection management
		self.detection_color_label.setEnabled(True)
		self.detection_color_setting.setEnabled(True)

		# Display type
		self.display_type_group.setEnabled(True)

	def calibration_interface(self, stage):
		'''
		Freezes parts of the interface that could interfere with an ongoing
		calibration and sets the detection frame, averaging setting, and
		HSV values based on incomming calibration data.
		'''
		# Disable parts of the interface that could interfere with the calibration
		if (stage == "capture"):

			# Uses the no loaded file interface as a base
			self.no_file_interface()

			# Display type
			self.display_type_group.setEnabled(True)

			# Holds the detection color so it can be restored after calibration
			self.detection_color_hold = self.detection_color
			self.calibrate_button.setEnabled(True)

		# Update relevant areas of the interface to display ongoing calibration data
		elif (stage == "update"):
			pass

		# Enable the interface to the extent that it was enabled before calibration
		elif (stage == "release"):

			# Restores the detection color setting that was stored earlier
			self.detection_color = self.detection_color_hold

			# Enable parts of the intrerface that were enabled before calibration
			self.file_loaded_interface()
			self.update_selection_color()
			self.update_detection_color()

	def save_file(self):
		'''
		Saves the calibration file using the save method in it's class object.
		'''
		self.calibration_file.save()
		self.status_bar.showMessage("File has been saved to %s" % (self.calibration_file.file))

	def load_file(self, mode):
		'''
		Used to create or load a file using the QtGui file browser with the
		default home directory set to the directory that the script was run
		from. Makes sure that no detection or selection image is being
		displayed before reloading the color list.
		'''
		if (mode == "new"):
			file = QtGui.QFileDialog.getSaveFileName(self.central_widget, "New Calibration File", os.getcwd(), "YAML (*.yaml);;All files (*)")
		elif (mode == "load"):
			file = QtGui.QFileDialog.getOpenFileName(self.central_widget, "Load Calibration File", os.getcwd(), "YAML (*.yaml);;All files (*)")

		if (file):

			# Terminates the displaying of selection or detection frames
	 		self.display.loop_selection = False
	 		if (self.detection_color == self.selection_color):
	 			self.display.loop_detection = False

			# Will not delete the calibration until both displays stop using it
	 			while (self.detection_thread.isAlive()):
	 				continue
	 		while (self.selection_thread.isAlive()):
	 			continue

			# Clears the image frame if no color is selected
	 		if (self.selection_color == ""):
	 			self.selection_frame.clear()
	 		if (self.detection_color == ""):
	 			self.detection_frame.clear()

	 		# Prevents loading files that do not exist
	 		if (mode == "load" and not os.path.isfile(file)):
				mode = "new"

			self.calibration_file = CalibrationData(file, mode)
			self.update_color_list()
			self.file_loaded_interface()

			# Prints the message relevant to the operation performed
			if (mode == "new"):
				self.status_bar.showMessage("New file has been created at %s" % (self.calibration_file.file))
			elif (mode == "load"):
				self.status_bar.showMessage("File has been loaded from %s" % (self.calibration_file.file))

	def update_frame(self, frame, display_to):
		'''
		Updates the frame displayed in one of the two display areas.
		'''
		# Reformats the image as a pixmap
		image = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
		pixmap = QtGui.QPixmap.fromImage(image)

		# Embeds the pixmap into the selected label object
		if (display_to == "selection"):
			if (not self.selection_color == ""):
				self.selection_frame.setPixmap(pixmap)
		if (display_to == "detection"):
			if (not self.detection_color == ""):
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

		for color in self.calibration_file.available_colors:
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
		Updates the selection_color variable, manages the selection frame
		display thread, and sets the slider and spin box values for the
		set color.
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

			# Enable selection management
			self.selection_managment_group.setEnabled(True)

			# Enable overlap prevention
			self.overlap_prevention_group.setEnabled(True)

			# Enable calibration button
			self.detection_color_label.setEnabled(True)
			self.calibrate_button.setEnabled(True)

			# Set the slider values to those stored for the selection color
			self.x1_slider.setValue(self.calibration_file.selection_boxes[self.selection_color][0][0])
			self.x2_slider.setValue(self.calibration_file.selection_boxes[self.selection_color][1][0])
			self.y1_slider.setValue(self.calibration_file.selection_boxes[self.selection_color][0][1])
			self.y2_slider.setValue(self.calibration_file.selection_boxes[self.selection_color][2][1])

		else:
			self.selection_color = ""

			# If no selection color is set, disable frame display
			try:
				self.display.loop_selection = False
			except:
				pass

			# Disable selection management
			self.selection_managment_group.setEnabled(False)

			# Disable overlap prevention
			self.overlap_prevention_group.setEnabled(False)

			# Disable calibration button
			self.detection_color_label.setEnabled(False)
			self.calibrate_button.setEnabled(False)

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
			self.color_ranges_group.setEnabled(True)
			self.load_hsv_boxes()

		else:
			self.detection_color = ""

			# If no selection color is set, disable frame display
			try:
				self.display.loop_detection = False
			except:
				pass

			# Clear and disable the HSV value display
			self.color_ranges_group.setEnabled(False)
			self.minimum_hsv_text.clear()
			self.maximum_hsv_text.clear()

	def clear_new_name(self):
		'''
		Clears the box for setting the name of a new calibration.
		'''
		if ((self.new_color_name_setting.text() == "Input a name to generate a new calibration") and (self.new_msg_printed == False)):
			self.new_color_name_setting.clear()

		# If the default new name message was just printed, do not clear it
		elif(self.new_msg_printed == True):
			self.new_msg_printed = False

	def print_new_name_msg(self):
		'''
		Prints the default new name text "Input a name to generate a new
		calibration" into the new calibration name text box.
		'''
		# Ensures that the default new name message will not be cleared
		self.new_msg_printed = True

		self.new_color_name_setting.end(False)
		if (self.new_color_name_setting.cursorPosition() == 0):
			self.new_color_name_setting.setText("Input a name to generate a new calibration")

	def create_new_calibration(self):
		'''
		Creates a new calibration in the calibration object based on the
		alphanumeric name specified. All calibration values will initialize to
		a value of 0.
		'''
		name = str(self.new_color_name_setting.text())

		# Ensures that a name has been typed into the new name box
		if (self.new_color_name_setting.text() == "Input a name to generate a new calibration"):
			self.status_bar.showMessage("ERROR: A name must be specified for the new calibration")

		# Ensures that the string recieved is alphanumeric
		elif (str.isalnum(name)):

			# Ensures that a unique name has been specified
			if (name in self.calibration_file.available_colors):
				self.status_bar.showMessage("ERROR: A calibration for this color already exists")
			else:

				# Initializes the new calibration an updates relevant lists
				self.calibration_file.hsv_ranges[name] = [[0, 0, 0], [0, 0, 0]]
				self.calibration_file.selection_boxes[name] = [[0, 0], [0, 0], [0, 0], [0, 0]]
				self.calibration_file.overlap_prevention_rules[name] = []
				self.calibration_file.update()
				self.update_color_list()
				self.status_bar.showMessage("A new calibration has been created for '%s'" % (name))

		else:
			self.status_bar.showMessage("ERROR: The entered name is not alphanumeric")

	def delete_calibration(self):
		'''
		Removes a calibration from memory only. Also updates the list of
		available colors on the user interface.
		'''
		if not (self.selection_color == ""):

			# Terminates the displaying of a color that is to be deleted
 			self.display.loop_selection = False
 			if (self.detection_color == self.selection_color):
 				self.display.loop_detection = False

 			# Will not delete the calibration until both displays stop using it
 				while (self.detection_thread.isAlive()):
 					continue
 			while (self.selection_thread.isAlive()):
 				continue

			# Deletes the color from the calibration object
			self.calibration_file.delete(self.colors[self.selection_color_setting.currentIndex()])
			self.status_bar.showMessage("The calibration for '%s' has been deleted" % (self.selection_color))
			self.update_color_list()

 			# Clears the image frame if no color is selected
 			if (self.selection_color == ""):
 				self.selection_frame.clear()
 			if (self.detection_color == ""):
 				self.detection_frame.clear()

		# Prevents the deletion of a null selection
		else:
			self.status_bar.showMessage("ERROR: Cannot delete a nonexistant calibration")

	def update_x1_coordinate(self, value):
		'''
		Updates the X1 point of the selection box.
		'''
		if (self.selection_color):
			self.calibration_file.selection_boxes[self.selection_color][0][0] = value
			self.calibration_file.selection_boxes[self.selection_color][3][0] = value

	def update_y1_coordinate(self, value):
		'''
		Updates the Y1 point of the selection box.
		'''
		if (self.selection_color):
			self.calibration_file.selection_boxes[self.selection_color][0][1] = value
			self.calibration_file.selection_boxes[self.selection_color][1][1] = value

	def update_x2_coordinate(self, value):
		'''
		Updates the X2 point of the selection box.
		'''
		if (self.selection_color):
			self.calibration_file.selection_boxes[self.selection_color][1][0] = value
			self.calibration_file.selection_boxes[self.selection_color][2][0] = value

	def update_y2_coordinate(self, value):
		'''
		Updates the Y2 point of the selection box.
		'''
		if (self.selection_color):
			self.calibration_file.selection_boxes[self.selection_color][2][1] = value
			self.calibration_file.selection_boxes[self.selection_color][3][1] = value

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

	def load_hsv_boxes(self):
		'''
		Prints the maximum and minimum HSV ranges for the set detection color
		to their respective text boxes. If no color is selected, clears the
		box. Also used to reset the ranges if changes are undone by the user.
		'''
		if (self.detection_color):
			minimum_values = self.calibration_file.hsv_ranges[self.detection_color][0]
			maximum_values = self.calibration_file.hsv_ranges[self.detection_color][1]
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
			self.status_bar.showMessage("ERROR: One of the HSV fields is blank")
			return None
		elif (minimum[0] == '(' and maximum[0] == '(' and minimum[len(minimum) - 1] == ')' and maximum[len(maximum) - 1] == ')'):
			try:
				minimum_values = list(int(char) for char in minimum[1:-1].split(','))
				maximum_values = list(int(char) for char in maximum[1:-1].split(','))
			except:
				self.status_bar.showMessage("ERROR: The values entered must be integers")
				return None
		else:
			self.status_bar.showMessage("ERROR: One or more of the HSV ranges is not formatted properly")
			return None

		# Ensures that the HSV values are within the range limits
		for value in minimum_values:
			if not (value >= 0 and value <= 255):
				self.status_bar.showMessage("ERROR: One or more of the HSV values is out of the expected range of [0, 255]")
				return None
		for value in maximum_values:
			if not (value >= 0 and value <= 255):
				self.status_bar.showMessage("ERROR: One or more of the HSV values is out of the expected range of [0, 255]")
				return None

		# Saves the values to the calibration object
		self.calibration_file.hsv_ranges[self.detection_color][0] = minimum_values
		self.calibration_file.hsv_ranges[self.detection_color][1] = maximum_values
		self.status_bar.showMessage("The HSV value ranges for '%s' have been saved to memory" % (self.detection_color))


		# Updates the available colors list and the numpy calibration values for cv2
		self.calibration_file.update()

	def closeEvent(self, event):
		'''
		Used to ensure clean termination of all threads when the main window is
		closed.
		'''
		self.display.loop_selection = False
		self.display.loop_detection = False
		QtCore.QCoreApplication.instance().quit()


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
		Draws the stored bounding box for the set selection color on a raw
		image frame. Emits a signal to update the detection frame in the GUI.
		'''
		# Separate image manipulation and detection objects for this frame
		camera.activate()
		image = Image(camera, self.gui.calibration_file, 480)
		detect = ObjectDetection(self.camera, image)

		while (self.loop_selection == True):
			color = self.gui.selection_color

			if (color):
				# Draw the selection bounding box and center point
				image.resize()
				detect.boxes[color] = self.gui.calibration_file.selection_boxes[color]
				detect.get_box_center()

				# Draw the detection bounding box and center point if one exists
				detect.draw_box([color], (255, 255, 255))
				detect.draw_center_point([color], (255, 255, 255))

				# Reformat the image to RGB, which is what QImage takes, and emit an update signal
				image.reformat_to_rgb()
				self.selection_frame = image.frame
				self.selection_frame_updated.emit()

				# Keep the refresh rate at or below 20 FPS
				time.sleep(1.0 / 20)

			# Clears the image if no color is selected
			else:
				gui.detection_frame.clear()

		# Cleanly disables the camera when it is no longer in use
		if (not self.loop_selection and not self.loop_detection):
			camera.deactivate()

	def get_detection_frame(self):
		'''
		Detects the set detection color and draws a bounding box and center
		point for it on the requested frame type. Emits a signal to update the
		detection frame in the GUI.
		'''
		# Separate image manipulation and detection objects for this frame
		camera.activate()
		image = Image(camera, self.gui.calibration_file, 480)
		detect = ObjectDetection(self.camera, image)

		while (self.loop_detection == True):
			color = self.gui.detection_color
			averaging = self.gui.averaging

			if (color):
				# Average detection of the selected color to obtain box and center points
				detect.average_box(detect.select_largest_object, [color], averaging)
				detect.get_box_center()

				# Pull the requested frame to display the image on
				image.hold_redux_frame = True
				if (self.gui.display_frame == "unfiltered"):
					image.resize()
				elif (self.gui.display_frame == "reduced"):
					image.reduce_colors(16)
				elif (self.gui.display_frame == "extracted"):
					image.extract_color(self.gui.detection_color)
				image.hold_redux_frame = False

				# Draw the detection bounding box and center point if one exists
				detect.draw_box([color], (255, 255, 255))
				detect.draw_center_point([color], (255, 255, 255))

				# Reformat the image to RGB, which is what QImage takes, and emit an update signal
				image.reformat_to_rgb()
				self.detection_frame = image.frame
				self.detection_frame_updated.emit()

				# Keep the refresh rate at or below 20 FPS
				time.sleep(1.0 / 20)

			# Clears the image if no color is selected
			else:
				gui.detection_frame.clear()

		# Cleanly disables the camera when it is no longer in use
		if (not self.loop_selection and not self.loop_detection):
			camera.deactivate()


if __name__ == '__main__':
	# Creating a camera object and launching the application
	camera = Camera(20)
	app = QtGui.QApplication(sys.argv)
	gui = ColorCalibrator()
	sys.exit(app.exec_())
