#!/usr/bin/python2
#==============================================================================
# Project: Machine Vision - Color Detection
# Module: Calibrator													v2.0
#
# Author: Anthony Olive	<anthony@iris-systems.net>
#==============================================================================

from PyQt4 import QtCore, QtGui
from mercurial.util import gui
import os
import sys
import threading
import time

from calibrator_gui import Ui_MainWindow
from camera import Camera
from color_calibration import CalibrationFile, ColorCalibrator
from detection import Image, ObjectDetector


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


class Calibrator(QtGui.QMainWindow, Ui_MainWindow):
	'''
	This class contains all of the top-level basic functionality of the GUI.
	All of the manager and pane objects are created within it and it is passed
	to them as an argument. This enables all of them to have access to the core
	functionality, variables, and signals needed to operate.
	'''

	def __init__(self, camera):
		self.camera = camera
		self.colors = {}
		self.__hold_values = False

		# The dimensions of a display frame on the GUI
		self.__image_shape = [480, 270]

		# Initialization of the GUI object tree
		super(self.__class__, self).__init__()
		self.setupUi(self)

		# Setup of manager objects for different parts of the interface
		self.__color_manager = ColorManager(self)
		self.__selection_manager = SelectionManager(self)
		self.__overlap_prevention_manager = OverlapPreventionManager(self)
		self.__range_manager = RangeManager(self)
		self.__calibration_manager = CalibrationManager(self, self.__color_manager.colors_updated)
		self.__selection_pane = SelectionPane(self, self.__color_manager, self.__selection_manager, self.__overlap_prevention_manager, self.__calibration_manager)
		self.__detection_pane = DetectionPane(self, self.__range_manager)
		self.disable_interface()

		# Enable menu file manipulation and program quitting
		self.menu_new.triggered.connect(lambda: self.__load_file("new"))
		self.menu_save.triggered.connect(self.__save_file)
		self.menu_load.triggered.connect(lambda: self.__load_file("load"))
		self.menu_quit.triggered.connect(self.closeEvent)

		# Connect color settings to the functions to update colors
		self.__color_manager.colors_updated.connect(self.__update_colors)
		self.__color_manager.stop_displaying_color.connect(self.__stop_displaying_color)
		self.selection_color_setting.activated.connect(self.set_selection_color)
		self.detection_color_setting.activated.connect(self.set_detection_color)

		# Links the pane getter functions to the ones in their panes
		self.get_selection_color = self.__selection_pane.get_color
		self.get_detection_color = self.__detection_pane.get_color
		self.get_averaging = self.__detection_pane.get_averaging
		self.get_display_type = self.__detection_pane.get_display_type

	def set_selection_color(self, index):
		'''
		Sets the selection color based on its GUI button.
		'''
		if (index >= 0):
			color = self.colors[index]
		else:
			color = ""

		self.selection_color_setting.setCurrentIndex(index)
		self.__selection_pane.set_color(color)

	def set_detection_color(self, index):
		'''
		Sets the detection color based on its GUI button.
		'''
		if (index >= 0):
			color = self.colors[index]
		else:
			color = ""

		self.detection_color_setting.setCurrentIndex(index)
		self.__detection_pane.set_color(color)

	def get_image_shape(self, parameter = None):
		'''
		Returns the shape (width, height, or both) of a display frame on the
		GUI.
		'''
		if (parameter == "width"):
			return self.__image_shape[0]
		elif (parameter == "height"):
			return self.__image_shape[1]
		else:
			return self.__image_shape

	def get_hold_value(self, state):
		'''
		Returns whether or not to hold a value instead of clearing it upon
		being disabled.
		'''
		return self.__hold_values

	def set_hold_value(self, state):
		'''
		Makes certain GUI objects hold their values upon being disabled when
		set to true as opposed to clearing them when set to false. The objects
		aforementioned objects are the selection sliders, new color name
		setting, and the selection frame display stream.
		'''
		self.__hold_values = state

	def disable_interface(self):
		'''
		Disables all components of the GUI via their managers.
		'''
		self.menu_save.setEnabled(False)
		self.calibrate_button.setEnabled(False)
		self.__selection_pane.disable()
		self.__detection_pane.disable()
		self.__color_manager.disable()

	def file_loaded_interface(self):
		'''
		Enables all GUI objects that are not dependent on colors being set.
		'''
		self.menu_save.setEnabled(True)
		self.__selection_pane.enable()
		self.__color_manager.enable()
		self.__detection_pane.enable()

	def __load_file(self, mode):
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
	 		self.__selection_pane.stop_display()
			self.calibration_file = CalibrationFile(file, mode, self.__image_shape)
 			self.__update_colors()
 			self.__calibration_manager.reload()
			self.file_loaded_interface()

			# Prints the message relevant to the operation performed
			if (mode == "new"):
				self.status_bar.showMessage("New file has been created at %s" % (self.calibration_file.get_path()))
			elif (mode == "load"):
				self.status_bar.showMessage("File has been loaded from %s" % (self.calibration_file.get_path()))

	def __save_file(self):
		'''
		Saves the calibration file using the save method in it's class object.
		'''
		self.calibration_file.save()
		self.status_bar.showMessage("File has been saved to %s" % (self.calibration_file.get_path()))

	def __update_colors(self):
		'''class inheritance
		Runs when the available colors are changed. Updates both of the color
		setting boxes and the overlap prevention list.
		'''
		item = 0

		# Clears the existing items in the relevant objects
		self.selection_color_setting.clear()
		self.detection_color_setting.clear()
		self.colors = {}

		for color in self.calibration_file.get_available_colors():

			# Adds an entry to both of the color setting boxes for each color
			self.selection_color_setting.addItem(_fromUtf8(""))
			self.selection_color_setting.setItemText(item, _translate("MainWindow", color, None))
			self.detection_color_setting.addItem(_fromUtf8(""))
			self.detection_color_setting.setItemText(item, _translate("MainWindow", color, None))

			# A dictionary storing which index each color is stored under
			self.colors[item] = color
			item += 1

		# Selects previously set selection color from index if available
		for index in range(len(self.colors)):
			if (self.colors[index] == self.get_selection_color()):
				self.selection_color_setting.setCurrentIndex(index)
				break
			else:
				self.selection_color_setting.setCurrentIndex(-1)
		self.set_selection_color(self.selection_color_setting.currentIndex())

		# Selects previously set detection color from index if available
		for index in range(len(self.colors)):
			if (self.colors[index] == self.get_detection_color()):
				self.detection_color_setting.setCurrentIndex(index)
				break
			else:
				self.detection_color_setting.setCurrentIndex(-1)
		self.set_detection_color(self.detection_color_setting.currentIndex())

	def __stop_displaying_color(self, color):
		if (self.get_selection_color() == color):
			self.__selection_pane.stop_display()
		if (self.get_detection_color() == color):
			self.__detection_pane.stop_display()

	def closeEvent(self, event):
		'''
		Used to ensure clean termination of all threads and the camera when the
		main window is closed.
		'''
		self.__selection_pane.stop_display()
		self.__detection_pane.stop_display()
		self.camera.deactivate()
		QtCore.QCoreApplication.instance().quit()


class Pane(QtGui.QMainWindow):
	'''
	This is the base class for both panes. It contains shared functions such
	as returning color values and managing the frame display.
	'''
	frame_updated = QtCore.pyqtSignal()

	def __init__(self, display_frame, stream_frames):
		super(Pane, self).__init__()
		self.is_enabled = False
		self.is_displaying = False
		self.color = ""
		self.frame = None

		# Values specific to the child object
		self.__display_frame = display_frame
		self.__stream_frames = stream_frames

		# Links the update signal to the frame updater
		self.frame_updated.connect(self.__update_frame)

	def get_color(self):
		'''
		Returns the currently selected detection color.
		'''
		return self.color

	def start_display(self):
		'''
		Starts a new thread to display selection frames from the camera if they
		are not already being displayed.
		'''
		if (self.is_enabled and not self.is_displaying):
			self.__display_thread = threading.Thread(target = self.__stream_frames)
			self.__display_thread.daemon = True
			self.__display_thread.start()
			self.is_displaying = True

	def stop_display(self):
		'''
		Stops the frame display thread if it is running. Will wait until it
		terminates and clear the last frame.
		'''
		if (self.is_displaying):
			self.is_displaying = False
	 		while (self.__display_thread.isAlive()):
	 			time.sleep(0.0001)
	 	self.__display_frame.clear()

	def __update_frame(self):
		'''
		Updates the frame displayed in one of the two display areas.
		'''
		if (self.is_displaying):

			# Reformats the image as a pixmap
			image = QtGui.QImage(self.frame, self.frame.shape[1], self.frame.shape[0], QtGui.QImage.Format_RGB888)
			pixmap = QtGui.QPixmap.fromImage(image)

			# Embeds the pixmap into the selected label object
	 		self.__display_frame.setPixmap(pixmap)


class SelectionPane(Pane):
	'''
	Manages the selection side of the GUI. Contains the ColorManager,
	SelectionManager, and OverlapPreventionManager objects and functions
	related to object detection.
	'''
	def __init__(self, gui, color_manager, selection_manager, overlap_prevention_manager, calibration_manager):
		self.__gui = gui
		super(self.__class__, self).__init__(self.__gui.selection_frame, self.__stream_frames)

		# Objects that control a group in this pane
		self.__color_manager = color_manager
		self.__selection_manager = selection_manager
		self.__overlap_prevention_manager = overlap_prevention_manager
		self.__calibration_manager = calibration_manager

	def enable(self):
		'''
		Enables the top-level objects on the detection pane on the GUI.
		'''
		self.__gui.selection_color_label.setEnabled(True)
		self.__gui.selection_color_setting.setEnabled(True)
		self.__gui.selection_frame.setEnabled(True)

		self.is_enabled = True

	def disable(self):
		'''
		Disables the top-level objects on the detection pane on the GUI.
		'''
		self.__gui.selection_color_label.setEnabled(False)
		self.__gui.selection_color_setting.setEnabled(False)
		self.__gui.selection_frame.setEnabled(False)

		if (not self.__gui.get_hold_value):
			self.stop_display()

		self.__selection_manager.disable()
		self.__overlap_prevention_manager.disable()
		self.__calibration_manager.disable()

		self.is_enabled = False

	def set_color(self, color):
		'''
		Updates the local color, manages the selection frame display thread,
		and initializes the color ColorManager, SelectionManager, and
		OverlapPreventionManager objects based on the set detection color.
		'''
		if (self.is_enabled and color):
			self.color = color
			self.start_display()
			self.__selection_manager.enable()
			self.__overlap_prevention_manager.enable()
			self.__calibration_manager.enable()

		else:
			self.__selection_manager.disable()
			self.__overlap_prevention_manager.disable()
			self.__calibration_manager.disable()
			self.stop_display()
			self.color = ""

	def __stream_frames(self):
		'''
		Draws the stored bounding box for the set selection color on a raw
		image frame. Emits a signal to update the detection frame in the GUI.
		'''
		# Separate image manipulation and detection objects for this frame
		self.__gui.camera.activate()
		image = Image(self.__gui.camera, self.__gui.calibration_file, self.__gui.get_image_shape("width"))

		while (self.is_displaying):
			color = self.color

			if (color):
				image.resize()

				# Draw the selection bounding box and center point if one exists
				points = self.__gui.calibration_file.get_selection_box(color)
				selection_box = [[points[0], points[2]], [points[0], points[3]], [points[1], points[3]], [points[1], points[2]]]
				image.draw_box(selection_box)
				image.draw_point(self.__get_box_center(selection_box))

				# Reformat the image to RGB, which is what QImage takes, and emit an update signal
				self.frame = image.reformat_to_rgb()
				self.frame_updated.emit()

				# Keep the refresh rate at or below 20 FPS
				time.sleep(1.0 / 20)

			# Clears the image if no color is selected
			else:
				self.__gui.selection_frame.clear()

	def __get_box_center(self, box):
		'''
		Finds the center point of the specified box by averaging the (x, y)
		points of the corners.
		'''
		box_avg = [0, 0]

		for point in range(4):
			for value in range(2):
				box_avg[value] = box_avg[value] + box[point][value]
		for value in range(2):
			box_avg[value] = int(box_avg[value] / len(box))
		return tuple(box_avg)


class DetectionPane(Pane):
	'''
	Manages the detection side of the GUI. Contains the RangeManager object and
	functions related to object detection.
	'''
	def __init__(self, gui, range_manager):
		self.__gui = gui
		super(self.__class__, self).__init__(self.__gui.detection_frame, self.__stream_frames)

		# Detection specific parameters
		self.__averaging = 2
		self.__display_type = "unfiltered"

		# Link the averaging slider to the averaging variable
		self.__gui.averaging_setting.valueChanged[int].connect(self.__set_averaging)

		# Link the display type selectors to the display_frame variable
		self.__gui.display_unfiltered_button.clicked.connect(self.__set_display_type)
		self.__gui.display_reduced_button.clicked.connect(self.__set_display_type)
		self.__gui.display_extracted_button.clicked.connect(self.__set_display_type)

		# Objects that control a group in this pane
		self.__range_manager = range_manager

	def enable(self):
		'''
		Enables the top-level objects on the detection pane on the GUI.
		'''
		self.__gui.detection_color_label.setEnabled(True)
		self.__gui.detection_color_setting.setEnabled(True)
		self.__gui.selections_to_average_label.setEnabled(True)
		self.__gui.averaging_setting.setEnabled(True)
		self.__gui.display_type_group.setEnabled(True)
		self.__gui.detection_frame.setEnabled(True)

		self.is_enabled = True

	def disable(self):
		'''
		Disables the top-level objects on the detection pane on the GUI.
		'''
		self.__gui.detection_color_label.setEnabled(False)
		self.__gui.detection_color_setting.setEnabled(False)
		self.__gui.selections_to_average_label.setEnabled(False)
		self.__gui.averaging_setting.setEnabled(False)
		self.__gui.display_type_group.setEnabled(False)
		self.__gui.detection_frame.setEnabled(False)

		self.stop_display()

		self.__range_manager.disable()

		self.is_enabled = False

	def set_color(self, color):
		'''
		Updates the local color, manages the detection frame display thread,
		and initializes the color RangeManager object based on the set
		detection color.
		'''
		if (self.is_enabled and color):
			self.color = color
			self.start_display()
			self.__range_manager.enable()

		else:
			self.__range_manager.disable()
			self.stop_display()
			self.color = ""

	def get_averaging(self):
		'''
		Returns the amount of frames that are currently being averaged for
		detection.
		'''
		return self.__averaging

	def get_display_type(self):
		'''
		Returns the type of frame that has been specified for displaying
		detection frames.
		'''
		return self.__display_type

	def __set_averaging(self, value):
		'''
		Set the averaging value based on it's spin box's setting.
		'''
		self.__averaging = value

	def __set_display_type(self):
		'''
		Sets the frame to display on the detection side of the GUI.
		'''
		if self.__gui.display_unfiltered_button.isChecked():
			self.__display_type = "unfiltered"
		if self.__gui.display_reduced_button.isChecked():
			self.__display_type = "reduced"
		if self.__gui.display_extracted_button.isChecked():
			self.__display_type = "extracted"

	def __stream_frames(self):
		'''
		Detects the set detection color and draws a bounding box and center
		point for it on the requested frame type. Emits a signal to update the
		detection frame in the GUI.
		'''
		# Separate image manipulation and detection objects for this frame
		self.__gui.camera.activate()
		image = Image(self.__gui.camera, self.__gui.calibration_file, self.__gui.get_image_shape("width"))
		detector = ObjectDetector(image)

		while (self.is_displaying):
			color = self.color
			averaging = self.__averaging

			if (color):
				# Average detection of the selected color to obtain box and center points
				detector.average_box(detector.select_largest_solid, [color], averaging)
				center = detector.get_box_center(color)

				# Pull the requested frame to display the image on
				image.set_hold_reduced(True)
				if (self.__display_type == "unfiltered"):
					image.resize()
				elif (self.__display_type == "reduced"):
					image.reduce_colors(16)
				elif (self.__display_type == "extracted"):
					image.extract_color(self.color)
				image.set_hold_reduced(False)

				# Draw the detection bounding box and center point if one exists
				detector.draw_selection([color])

				# Reformat the image to RGB, which is what QImage takes, and emit an update signal
				self.frame = image.reformat_to_rgb()
				self.frame_updated.emit()

				# Keep the refresh rate at or below 20 FPS
				time.sleep(1.0 / 20)

			# Clears the image if no color is selected
			else:
				self.__gui.detection_frame.clear()


class ColorManager(QtCore.QObject):
	'''
	Manages the color calibrations in the calibration file. Can be used to
	create and delete colors.
	'''
	colors_updated = QtCore.pyqtSignal()
	stop_displaying_color = QtCore.pyqtSignal(str)

	def __init__(self, gui):
		super(ColorManager, self).__init__()

		self.__gui = gui
		self.__is_enabled = False
		self.__new_msg_printed = False

		# Link the calibration management buttons to the calibration functions
		self.__gui.new_color_name_setting.cursorPositionChanged.connect(self.__clear)
		self.__gui.new_color_name_setting.editingFinished.connect(self.__print_new_msg)
		self.__gui.new_color_name_setting.returnPressed.connect(self.__create_calibration)
		self.__gui.new_color_button.clicked.connect(self.__create_calibration)
		self.__gui.delete_color_button.clicked.connect(self.__delete_calibration)

	def enable(self):
		'''
		Enables the selection group on the GUI. Can also be used to reload the
		values upon change in the set color calibration.
		'''
		self.__gui.new_color_name_setting.setEnabled(True)
		self.__gui.new_color_button.setEnabled(True)
		self.__gui.delete_color_button.setEnabled(True)
		self.__is_enabled = True

		self.__print_new_msg()

	def disable(self):
		'''
		Disables the selection group on the GUI and clears the values of the
		sliders.
		'''
		self.__gui.new_color_name_setting.setEnabled(False)
		self.__gui.new_color_button.setEnabled(False)
		self.__gui.delete_color_button.setEnabled(False)
		self.__is_enabled = False

		if (not self.__gui.get_hold_value):
			self.__clear()

	def __clear(self):
		'''
		Clears the box for setting the name of a new calibration and clears the box.
		'''
		if ((self.__gui.new_color_name_setting.text() == "Input a name to generate a new calibration") and (self.__new_msg_printed == False)):
			self.__gui.new_color_name_setting.clear()

		# If the default new name message was just printed, do not clear it
		elif(self.__new_msg_printed):
			self.__new_msg_printed = False

	def __print_new_msg(self):
		'''
		Prints the default new name text "Input a name to generate a new
		calibration" into the new calibration name text box.
		'''
		# Ensures that the default new name message will not be cleared
		if (self.__is_enabled):
			self.__new_msg_printed = True

			self.__gui.new_color_name_setting.end(False)
			if (self.__gui.new_color_name_setting.cursorPosition() == 0):
				self.__gui.new_color_name_setting.setText("Input a name to generate a new calibration")

	def __create_calibration(self):
		'''
		Creates a new calibration in the calibration object based on the
		alphanumeric name specified. All HSV values will initialize to a value
		of 0 and the selection box will be over the center of the image.
		'''
		if (self.__is_enabled):
			name = str(self.__gui.new_color_name_setting.text())

			# Ensures that a name has been typed into the new name box
			if ((self.__gui.new_color_name_setting.text() == "") or (self.__gui.new_color_name_setting.text() == "Input a name to generate a new calibration")):
				self.__gui.status_bar.showMessage("ERROR: A name must be specified for the new calibration")

			# Ensures that the string recieved is alphanumeric
			elif (str.isalnum(name)):

				# Ensures that a unique name has been specified
				if (name in self.__gui.calibration_file.get_available_colors()):
					self.__gui.status_bar.showMessage("ERROR: A calibration for this color already exists")

				# Creates the color calibration and updates the list of colors
				else:
					self.__gui.calibration_file.create_color(name)
					self.__gui.status_bar.showMessage("A new calibration has been created for '%s'" % (name))
					self.__gui.new_color_name_setting.setText("Input a name to generate a new calibration")
					self.colors_updated.emit()

			else:
				self.__gui.status_bar.showMessage("ERROR: The entered name is not alphanumeric")

	def __delete_calibration(self):
		'''
		Removes a calibration from memory only. Also emits a signal to update
		the list of available colors on the user interface.
		'''
		if (self.__is_enabled):
			if not (self.__gui.get_selection_color() == ""):
				self.stop_displaying_color.emit(self.__gui.get_selection_color())

			 	# Ensures that the calibration exists before attempting to remove it
				if (not self.__gui.get_selection_color() in self.__gui.calibration_file.get_available_colors()):
					self.__gui.status_bar.showMessage("No color calibration exists for '%s'" % (color))

				# Deletes the color and updates the available colors if successful
				else:
					self.__gui.calibration_file.delete_color(self.__gui.get_selection_color())
					self.__gui.status_bar.showMessage("The calibration for '%s' has been deleted" % (self.__gui.get_selection_color()))
					self.colors_updated.emit()

			# Prevents the deletion of a null selection
			else:
				self.__gui.status_bar.showMessage("ERROR: Cannot delete a nonexistant calibration")


class SelectionManager():
	'''
	Manages the selection box settings for the set color by changing them
	to reflect the state of the box point position sliders boxes in the GUI.
	'''
	def __init__(self, gui):
		self.__gui = gui
		self.__is_enabled = False

		# Link the X and Y coordinate sliders to their set functions
		self.__gui.x1_slider.valueChanged[int].connect(self.__set_x1)
		self.__gui.y1_slider.valueChanged[int].connect(self.__set_y1)
		self.__gui.x2_slider.valueChanged[int].connect(self.__set_x2)
		self.__gui.y2_slider.valueChanged[int].connect(self.__set_y2)

	def enable(self):
		'''
		Enables the selection group on the GUI. Can also be used to reload the
		values upon change in the set color calibration.
		'''
		self.__gui.selection_managment_group.setEnabled(True)
		self.__is_enabled = True

		self.__set_selection_sliders()

	def disable(self):
		'''
		Disables the selection group on the GUI and clears the values of the
		sliders.
		'''
		self.__gui.selection_managment_group.setEnabled(False)
		self.__is_enabled = False

		if (not self.__gui.get_hold_value):
			self.__clear()

	def __clear(self):
		'''
		Clears all slider and roll box values by setting them to zero
		'''
		self.__gui.x1_slider.setValue(0)
		self.__gui.x2_slider.setValue(0)
		self.__gui.y1_slider.setValue(0)
		self.__gui.y2_slider.setValue(0)

	def __set_x1(self, value):
		'''
		Sets the X1 point of the selection box.
		'''
		self.__set_coordinate(0, value)

	def __set_x2(self, value):
		'''
		Sets the X2 point of the selection box.
		'''
		self.__set_coordinate(1, value)

	def __set_y1(self, value):
		'''
		Sets the Y1 point of the selection box.
		'''
		self.__set_coordinate(2, value)

	def __set_y2(self, value):
		'''
		Sets the Y2 point of the selection box.
		'''
		self.__set_coordinate(3, value)

	def __set_coordinate(self, coordinate, value):
		'''
		General setter for all points in the selection box. Will attempt to set
		the selection box and reset the slider values if unsuccessful.
		'''
		if (self.__is_enabled):
			selection_box = list(self.__gui.calibration_file.get_selection_box(self.__gui.get_selection_color()))
			selection_box[coordinate] = value

			# Ensures that the selection is not too small
			width = abs(selection_box[1] - selection_box[0])
			height = abs(selection_box[3] - selection_box[2])
			area = width * height
			if (area < self.__gui.calibration_file.get_minimum_selection()):
				self.__gui.status_bar.showMessage("A selection of %d pixels is too small and it will be reset" % (area))
				self.__set_selection_sliders()

			else:
				self.__gui.calibration_file.set_selection_box(self.__gui.get_selection_color(), selection_box)

	def __set_selection_sliders(self):
		'''
		Set the slider values to those stored for the detection color.
		'''
		if (self.__is_enabled and self.__gui.get_selection_color()):
			selection_box = self.__gui.calibration_file.get_selection_box(self.__gui.get_selection_color())
			self.__gui.x1_slider.setValue(selection_box[0])
			self.__gui.x2_slider.setValue(selection_box[1])
			self.__gui.y1_slider.setValue(selection_box[2])
			self.__gui.y2_slider.setValue(selection_box[3])
		else:
			self.__gui.x1_slider.setValue(int(self.__gui.get_image_shape("width") * (1.0 / 3)))
			self.__gui.x2_slider.setValue(int(self.__gui.get_image_shape("width") * (2.0 / 3)))
			self.__gui.y1_slider.setValue(int(self.__gui.get_image_shape("height") * (1.0 / 3)))
			self.__gui.y2_slider.setValue(int(self.__gui.get_image_shape("height") * (2.0 / 3)))


class OverlapPreventionManager():
	'''
	Manages the overlap prevention settings for the set color by changing them
	to reflect the state of the overlap prevention check boxes in the GUI.
	'''
	def __init__(self, gui):
		self.__gui = gui
		self.__is_enabled = False

		# Stores the QCheckBox objects used to select rules and their colors
		self.__prevention_setting = {}
		self.__prevention_colors = []

	def enable(self):
		'''
		Enables the overlap prevention group in the GUI and loads the rules.
		Can also be used to reload the values upon change in the set color
		calibration.
		'''
		self.__gui.overlap_prevention_group.setEnabled(True)
		self.__is_enabled = True

		self.__generate_settings()

	def disable(self):
		'''
		Disables the overlap prevention group in the GUI and clears the
		prevention settings in the list box.
		'''
		self.__gui.overlap_prevention_group.setEnabled(False)
		self.__is_enabled = False

	def __clear(self):
		'''
		Clears all of the check boxes from the prevention settings list view
		and allows the garbage collector to destroy them.
		'''
		for setting in self.__prevention_setting:
			self.__prevention_setting[setting].setParent(None)
		self.__prevention_setting = {}
		self.__prevention_colors = []

	def __generate_settings(self):
		'''
		Generates the list view of overlap prevention check boxes based on the
		overlap prevention settings that exist in the calibrations for the
		selection color.
		'''
		if (self.__is_enabled):
			self.__clear()
			for color in self.__gui.calibration_file.get_available_colors():

				# Prevents the selected color from being added to the list
				if (not color == self.__gui.get_selection_color()):

					# Adds an entry to the overlap prevention scroll area for each color
					self.__prevention_setting[color] = QtGui.QCheckBox(self.__gui.overlap_prevention_colors)
					self.__prevention_setting[color].setObjectName(_fromUtf8("%s_prevention_setting" % (color)))
					self.__gui.verticalLayout_8.addWidget(self.__prevention_setting[color])
					self.__prevention_setting[color].setText(_translate("MainWindow", color, None))

					# Loads existing prevention settings from the calibrations
					if (color in self.__gui.calibration_file.get_overlap_prevention(self.__gui.get_selection_color())):
						self.__prevention_setting[color].setCheckState(2)
					else:
						self.__prevention_setting[color].setCheckState(0)

					# Connect the check boxes to a method that modifies the respective calibration values
					self.__prevention_colors.append(color)
					self.__prevention_setting[color].stateChanged.connect(self.__modify_setting)

	def __modify_setting(self):
		'''
		If the box has been checked, the color is added to the prevention list.
		If the box has been unchecked, the color is removed from the prevention
		list.
		'''
		if (self.__is_enabled):
			for color in self.__prevention_colors:

				# Adds an item to the calibration's prevention settings if it was checked
				if ((self.__prevention_setting[color].isChecked()) and (not color in self.__gui.calibration_file.get_overlap_prevention(self.__gui.get_selection_color()))):
					self.__gui.calibration_file.add_prevention_rule(self.__gui.get_selection_color(), color)

				# Removes an item from the calibration's prevention settings if it was unchecked
				elif ((not self.__prevention_setting[color].isChecked()) and (color in self.__gui.calibration_file.get_overlap_prevention(self.__gui.get_selection_color()))):
					self.__gui.calibration_file.remove_prevention_rule(self.__gui.get_selection_color(), color)


class RangeManager():
	'''
	Displays the HSV range values for the set color and allows them to be
	edited.
	'''
	def __init__(self, gui):
		self.__gui = gui
		self.__is_enabled = False

		# Link the HSV display buttons to their display set functions
		self.__gui.save_changes_button.clicked.connect(self.__save)
		self.__gui.undo_changes_button.clicked.connect(self.__load)
		self.__gui.minimum_hsv_text.returnPressed.connect(self.__save)
		self.__gui.maximum_hsv_text.returnPressed.connect(self.__save)

	def enable(self):
		'''
		Disables the HSV range group on the GUI. Can also be used to reload the
		values upon change in the set color calibration.
		'''
		self.__gui.color_ranges_group.setEnabled(True)
		self.__is_enabled = True

		self.__load()

	def disable(self):
		'''
		Disables the HSV range group on the GUI and clears the text boxes.
		'''
		self.__gui.color_ranges_group.setEnabled(False)
		self.__is_enabled = False

		self.__clear()

	def __clear(self):
		'''
		Clears the text in both HSV range boxes and disables the group.
		'''
		self.__gui.minimum_hsv_text.clear()
		self.__gui.maximum_hsv_text.clear()

	def __load(self):
		'''
		Prints the maximum and minimum HSV values for the set detection color
		to their respective text boxes. If no color is selected, clears the
		box. Also used to reset the ranges if changes are undone by the user.
		'''
		if (self.__is_enabled):
			minimum, maximum, average = self.__gui.calibration_file.get_hsv_range(self.__gui.get_detection_color())
			self.__gui.minimum_hsv_text.setText("(%d, %d, %d)" % (minimum[0], minimum[1], minimum[2]))
			self.__gui.maximum_hsv_text.setText("(%d, %d, %d)" % (maximum[0], maximum[1], maximum[2]))
			self.__gui.average_v_text.setText("%d" % (average))

	def __save(self):
		'''
		Saves the HSV values currently in the boxes to the calibrations object
		if they are properly formatted tupples within the acceptable range of
		[0, 256). NOTE: This does not save the values to the file!
		'''
		if (self.__is_enabled):
			minimum = self.__gui.minimum_hsv_text.text()
			maximum = self.__gui.maximum_hsv_text.text()
			average = int(self.__gui.average_v_text.text())

			# Determine if the tupple is properly formatted to (x, y, z)
			if not (minimum and maximum):
				self.__gui.status_bar.showMessage("ERROR: One of the HSV fields is blank")
				return None
			elif (minimum[0] == '(' and maximum[0] == '(' and minimum[len(minimum) - 1] == ')' and maximum[len(maximum) - 1] == ')'):
				try:
					minimum_values = list(int(char) for char in minimum[1:-1].split(','))
					maximum_values = list(int(char) for char in maximum[1:-1].split(','))
				except:
					self.__gui.status_bar.showMessage("ERROR: The values entered must be integers")
					return None
			else:
				self.__gui.status_bar.showMessage("ERROR: One or more of the HSV range values is not formatted properly")
				return None

			# Ensures that the HSV values are within the range limits
			for value in minimum_values:
				if (value < 0 or value >= 256):
					self.__gui.status_bar.showMessage("ERROR: One or more of the HSV values is out of the expected range of [0, 256)")
					return None
			for value in maximum_values:
				if (value < 0 or value >= 256):
					self.__gui.status_bar.showMessage("ERROR: One or more of the HSV values is out of the expected range of [0, 256)")
					return None

			if (type(average) != int):
					self.__gui.status_bar.showMessage("ERROR: The values entered must be integers")
					return None
			elif (average < 0 or average >= 256):
					self.__gui.status_bar.showMessage("ERROR: One or more of the HSV values is out of the expected range of [0, 256)")
					return None

			# Saves the values to the calibration object
			self.__gui.calibration_file.set_hsv_range(self.__gui.get_detection_color(), [minimum_values, maximum_values, average])
			self.__gui.status_bar.showMessage("The HSV value ranges for '%s' have been saved to memory" % (self.__gui.get_detection_color()))


class CalibrationManager(QtGui.QMainWindow):
	'''
	Used when calibrating for a specific color. Locks the interface and uses
	the display frame, HSV range display, and progress bar to show the ongoing
	status of the calibration.
	'''
	calibration_step = QtCore.pyqtSignal()

	def __init__(self, gui, colors_updated_signal):
		super(CalibrationManager, self).__init__()

		self.__gui = gui
		self.__colors_updated = colors_updated_signal
		self.__is_enabled = False

		# Variables used to manage the captured state and reset it afterwards
		self.__state = "released"
		self.__detection_color_cap = -1
		self.__averaging_cap = 2

		# Links the calibrate button to it's manager function
		self.__gui.calibrate_button.clicked.connect(self.__calibrate_clicked)

		# Links the calibration step signal to the display update function
		self.calibration_step.connect(self.update_display)

	def enable(self):
		'''
		Enables calibration functionality.
		'''
		self.__gui.calibrate_button.setEnabled(True)
		self.__gui.progress_bar.setEnabled(True)
		self.__is_enabled = True

	def disable(self):
		'''
		Disables calibration functionality.
		'''
		self.__gui.calibrate_button.setEnabled(False)
		self.__gui.progress_bar.setEnabled(False)
		self.__is_enabled = False

	def reload(self):
		'''
		Used to reload the actual calibration object if the calibration file
		is changed.
		'''
		self.__calibrator = ColorCalibrator(self.__gui.camera, self.__gui.calibration_file, self.__gui.get_image_shape("width"), self.__gui)

	def __calibrate_clicked(self):
		'''
		Manages the function of the calibrate button based on the current state
		of this object (i.e. captured or released)
		'''
		if (self.__is_enabled):
			if (self.__state == "captured"):
				self.__release()
			elif (self.__state == "released" and self.__calibrator):
				self.__capture()
				self.__calibrator.calibrate(self.__gui.get_selection_color())
			else:
				self.__gui.status_bar.showMessage("ERROR: Unable to calibrate because no calibration file loaded.")

	def __capture(self):
		'''
		Takes control of the GUI, any interface objects that could interfere
		with the running calibration.
		'''
		self.__gui.set_hold_value(True)

		self.__gui.disable_interface()
		self.enable()
		self.__gui.progress_bar.setTextVisible(True)
		self.__gui.display_type_group.setEnabled(True)

		# Storing variables to be restored after capture is released
		self.__detection_color_cap = self.__gui.detection_color_setting.currentIndex()
		self.__averaging_cap = self.__gui.get_averaging()

		# Managing the captured state
		self.__gui.calibrate_button.setText("Cancel")
		self.__state = "captured"

		# Sets the detection color and averaging parameters in the GUI
		self.__gui.detection_color_setting.setCurrentIndex(self.__gui.selection_color_setting.currentIndex())
		self.__gui.averaging_setting.setValue(self.__calibrator.get_averaging())

	def __release(self):
		'''
		Restores the values that were originally on the GUI and enables all of
		the objects that were active before the calibration began.
		'''
		self.__gui.set_hold_value(False)

		self.__gui.progress_bar.setTextVisible(False)
		self.__gui.detection_frame.clear()
		self.__gui.file_loaded_interface()
		self.__colors_updated.emit()

		# Restoring the captured variables
		self.__gui.detection_color_setting.setCurrentIndex(self.__detection_color_cap)
		self.__gui.set_detection_color(self.__detection_color_cap)
		self.__gui.averaging_setting.setValue(self.__averaging_cap)

		# Managing the captured state
		self.__gui.calibrate_button.setText("Calibrate")
		self.__state = "released"

	def update_display(self):
		'''
		Publishes data from the ongoing calibration to the HSV range boxes,
		detection frame, and progress bar.
		'''
		if (self.__is_enabled and (self.__state == "captured")):
			# Updates the HSV range based on what the calibrator doing with it
			minimum, maximum = self.__calibrator.get_step_range
			self.__gui.minimum_hsv_text.setText("(%d, %d, %d)" % (minimum[0], minimum[1], minimum[2]))
			self.__gui.maximum_hsv_text.setText("(%d, %d, %d)" % (maximum[0], maximum[1], maximum[2]))

			# Updates the progress bar based on the calibrator's state
			self.__gui.progress_bar.setValue(self.__calibrator.step_progress)

			# Retrieves a detection frame of the specified type for this step
			frame = self.__calibrator.step_display_frame

			# Reformats the image as a pixmap
			image = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
			pixmap = QtGui.QPixmap.fromImage(image)

			# Embeds the pixmap into the selected label object
	 		self.__gui.detection_frame.setPixmap(pixmap)


if __name__ == '__main__':
	camera = Camera()
	app = QtGui.QApplication(sys.argv)
	calibrator = Calibrator(camera)
	calibrator.show()
	sys.exit(app.exec_())
