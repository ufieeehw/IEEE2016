# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'color_calibrator_gui.ui'
#
# Created: Tue Mar 15 22:56:10 2016
#      by: PyQt4 UI code generator 4.11.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

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

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1010, 744)
        self.central_widget = QtGui.QWidget(MainWindow)
        self.central_widget.setObjectName(_fromUtf8("central_widget"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.central_widget)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.horizontal_layout_1 = QtGui.QHBoxLayout()
        self.horizontal_layout_1.setObjectName(_fromUtf8("horizontal_layout_1"))
        self.vertical_layout_1 = QtGui.QVBoxLayout()
        self.vertical_layout_1.setObjectName(_fromUtf8("vertical_layout_1"))
        self.color_selection_group = QtGui.QGroupBox(self.central_widget)
        self.color_selection_group.setObjectName(_fromUtf8("color_selection_group"))
        self.gridLayout = QtGui.QGridLayout(self.color_selection_group)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.selection_frame = QtGui.QLabel(self.color_selection_group)
        self.selection_frame.setMinimumSize(QtCore.QSize(320, 180))
        self.selection_frame.setMaximumSize(QtCore.QSize(320, 180))
        self.selection_frame.setText(_fromUtf8(""))
        self.selection_frame.setObjectName(_fromUtf8("selection_frame"))
        self.gridLayout.addWidget(self.selection_frame, 1, 1, 1, 1)
        spacerItem = QtGui.QSpacerItem(55, 20, QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem, 1, 2, 1, 1)
        spacerItem1 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.gridLayout.addItem(spacerItem1, 0, 1, 1, 1)
        spacerItem2 = QtGui.QSpacerItem(35, 20, QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem2, 1, 0, 1, 1)
        spacerItem3 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.gridLayout.addItem(spacerItem3, 2, 1, 1, 1)
        self.vertical_layout_1.addWidget(self.color_selection_group)
        self.horizontal_layout_2 = QtGui.QHBoxLayout()
        self.horizontal_layout_2.setObjectName(_fromUtf8("horizontal_layout_2"))
        self.selection_color_label = QtGui.QLabel(self.central_widget)
        self.selection_color_label.setObjectName(_fromUtf8("selection_color_label"))
        self.horizontal_layout_2.addWidget(self.selection_color_label)
        self.selection_color_setting = QtGui.QComboBox(self.central_widget)
        self.selection_color_setting.setObjectName(_fromUtf8("selection_color_setting"))
        self.horizontal_layout_2.addWidget(self.selection_color_setting)
        self.vertical_layout_1.addLayout(self.horizontal_layout_2)
        self.horizontal_layout_3 = QtGui.QHBoxLayout()
        self.horizontal_layout_3.setObjectName(_fromUtf8("horizontal_layout_3"))
        self.new_color_name_setting = QtGui.QLineEdit(self.central_widget)
        self.new_color_name_setting.setObjectName(_fromUtf8("new_color_name_setting"))
        self.horizontal_layout_3.addWidget(self.new_color_name_setting)
        self.new_color_button = QtGui.QPushButton(self.central_widget)
        self.new_color_button.setObjectName(_fromUtf8("new_color_button"))
        self.horizontal_layout_3.addWidget(self.new_color_button)
        self.delete_color_button = QtGui.QPushButton(self.central_widget)
        self.delete_color_button.setObjectName(_fromUtf8("delete_color_button"))
        self.horizontal_layout_3.addWidget(self.delete_color_button)
        self.vertical_layout_1.addLayout(self.horizontal_layout_3)
        self.selection_managment_group = QtGui.QGroupBox(self.central_widget)
        self.selection_managment_group.setObjectName(_fromUtf8("selection_managment_group"))
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.selection_managment_group)
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.point_selector = QtGui.QTabWidget(self.selection_managment_group)
        self.point_selector.setObjectName(_fromUtf8("point_selector"))
        self.tab_1 = QtGui.QWidget()
        self.tab_1.setObjectName(_fromUtf8("tab_1"))
        self.gridLayout_6 = QtGui.QGridLayout(self.tab_1)
        self.gridLayout_6.setObjectName(_fromUtf8("gridLayout_6"))
        self.x1_slider = QtGui.QSlider(self.tab_1)
        self.x1_slider.setMaximum(319)
        self.x1_slider.setOrientation(QtCore.Qt.Horizontal)
        self.x1_slider.setObjectName(_fromUtf8("x1_slider"))
        self.gridLayout_6.addWidget(self.x1_slider, 0, 1, 1, 1)
        self.x1_label = QtGui.QLabel(self.tab_1)
        self.x1_label.setObjectName(_fromUtf8("x1_label"))
        self.gridLayout_6.addWidget(self.x1_label, 0, 0, 1, 1)
        self.y1_slider = QtGui.QSlider(self.tab_1)
        self.y1_slider.setMaximum(179)
        self.y1_slider.setOrientation(QtCore.Qt.Horizontal)
        self.y1_slider.setObjectName(_fromUtf8("y1_slider"))
        self.gridLayout_6.addWidget(self.y1_slider, 1, 1, 1, 1)
        self.y1_label = QtGui.QLabel(self.tab_1)
        self.y1_label.setObjectName(_fromUtf8("y1_label"))
        self.gridLayout_6.addWidget(self.y1_label, 1, 0, 1, 1)
        self.x1_setting = QtGui.QSpinBox(self.tab_1)
        self.x1_setting.setMaximum(319)
        self.x1_setting.setObjectName(_fromUtf8("x1_setting"))
        self.gridLayout_6.addWidget(self.x1_setting, 0, 2, 1, 1)
        self.y1_setting = QtGui.QSpinBox(self.tab_1)
        self.y1_setting.setMaximum(179)
        self.y1_setting.setObjectName(_fromUtf8("y1_setting"))
        self.gridLayout_6.addWidget(self.y1_setting, 1, 2, 1, 1)
        self.point_selector.addTab(self.tab_1, _fromUtf8(""))
        self.tab_11 = QtGui.QWidget()
        self.tab_11.setObjectName(_fromUtf8("tab_11"))
        self.gridLayout_5 = QtGui.QGridLayout(self.tab_11)
        self.gridLayout_5.setObjectName(_fromUtf8("gridLayout_5"))
        self.y2_label = QtGui.QLabel(self.tab_11)
        self.y2_label.setObjectName(_fromUtf8("y2_label"))
        self.gridLayout_5.addWidget(self.y2_label, 3, 0, 1, 1)
        self.x2_label = QtGui.QLabel(self.tab_11)
        self.x2_label.setObjectName(_fromUtf8("x2_label"))
        self.gridLayout_5.addWidget(self.x2_label, 2, 0, 1, 1)
        self.x2_slider = QtGui.QSlider(self.tab_11)
        self.x2_slider.setMaximum(319)
        self.x2_slider.setOrientation(QtCore.Qt.Horizontal)
        self.x2_slider.setObjectName(_fromUtf8("x2_slider"))
        self.gridLayout_5.addWidget(self.x2_slider, 2, 1, 1, 1)
        self.y2_slider = QtGui.QSlider(self.tab_11)
        self.y2_slider.setMaximum(179)
        self.y2_slider.setOrientation(QtCore.Qt.Horizontal)
        self.y2_slider.setObjectName(_fromUtf8("y2_slider"))
        self.gridLayout_5.addWidget(self.y2_slider, 3, 1, 1, 1)
        self.x2_setting = QtGui.QSpinBox(self.tab_11)
        self.x2_setting.setMaximum(319)
        self.x2_setting.setObjectName(_fromUtf8("x2_setting"))
        self.gridLayout_5.addWidget(self.x2_setting, 2, 2, 1, 1)
        self.y2_setting = QtGui.QSpinBox(self.tab_11)
        self.y2_setting.setMaximum(179)
        self.y2_setting.setObjectName(_fromUtf8("y2_setting"))
        self.gridLayout_5.addWidget(self.y2_setting, 3, 2, 1, 1)
        self.point_selector.addTab(self.tab_11, _fromUtf8(""))
        self.verticalLayout_5.addWidget(self.point_selector)
        self.vertical_layout_1.addWidget(self.selection_managment_group)
        self.overlap_prevention_group = QtGui.QGroupBox(self.central_widget)
        self.overlap_prevention_group.setObjectName(_fromUtf8("overlap_prevention_group"))
        self.verticalLayout_9 = QtGui.QVBoxLayout(self.overlap_prevention_group)
        self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
        self.overlap_prevention_list = QtGui.QScrollArea(self.overlap_prevention_group)
        self.overlap_prevention_list.setWidgetResizable(True)
        self.overlap_prevention_list.setObjectName(_fromUtf8("overlap_prevention_list"))
        self.overlap_prevention_colors = QtGui.QWidget()
        self.overlap_prevention_colors.setGeometry(QtCore.QRect(0, 0, 458, 80))
        self.overlap_prevention_colors.setObjectName(_fromUtf8("overlap_prevention_colors"))
        self.verticalLayout_8 = QtGui.QVBoxLayout(self.overlap_prevention_colors)
        self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
        self.overlap_prevention_list.setWidget(self.overlap_prevention_colors)
        self.verticalLayout_9.addWidget(self.overlap_prevention_list)
        self.vertical_layout_1.addWidget(self.overlap_prevention_group)
        self.horizontal_layout_5 = QtGui.QHBoxLayout()
        self.horizontal_layout_5.setObjectName(_fromUtf8("horizontal_layout_5"))
        self.calibrate_button = QtGui.QPushButton(self.central_widget)
        self.calibrate_button.setObjectName(_fromUtf8("calibrate_button"))
        self.horizontal_layout_5.addWidget(self.calibrate_button)
        self.progress_bar = QtGui.QProgressBar(self.central_widget)
        self.progress_bar.setEnabled(False)
        self.progress_bar.setProperty("value", 0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setObjectName(_fromUtf8("progress_bar"))
        self.horizontal_layout_5.addWidget(self.progress_bar)
        self.vertical_layout_1.addLayout(self.horizontal_layout_5)
        spacerItem4 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.vertical_layout_1.addItem(spacerItem4)
        self.horizontal_layout_1.addLayout(self.vertical_layout_1)
        self.selection_detection_divider = QtGui.QFrame(self.central_widget)
        self.selection_detection_divider.setFrameShadow(QtGui.QFrame.Plain)
        self.selection_detection_divider.setLineWidth(1)
        self.selection_detection_divider.setFrameShape(QtGui.QFrame.VLine)
        self.selection_detection_divider.setFrameShadow(QtGui.QFrame.Sunken)
        self.selection_detection_divider.setObjectName(_fromUtf8("selection_detection_divider"))
        self.horizontal_layout_1.addWidget(self.selection_detection_divider)
        self.vertical_layout_2 = QtGui.QVBoxLayout()
        self.vertical_layout_2.setObjectName(_fromUtf8("vertical_layout_2"))
        self.object_detection_group = QtGui.QGroupBox(self.central_widget)
        self.object_detection_group.setObjectName(_fromUtf8("object_detection_group"))
        self.gridLayout_3 = QtGui.QGridLayout(self.object_detection_group)
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        spacerItem5 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.gridLayout_3.addItem(spacerItem5, 5, 1, 1, 1)
        spacerItem6 = QtGui.QSpacerItem(35, 20, QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem6, 3, 0, 1, 1)
        spacerItem7 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.gridLayout_3.addItem(spacerItem7, 0, 1, 1, 1)
        spacerItem8 = QtGui.QSpacerItem(55, 20, QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem8, 3, 2, 1, 1)
        self.detection_frame = QtGui.QLabel(self.object_detection_group)
        self.detection_frame.setMinimumSize(QtCore.QSize(320, 180))
        self.detection_frame.setMaximumSize(QtCore.QSize(320, 180))
        self.detection_frame.setText(_fromUtf8(""))
        self.detection_frame.setObjectName(_fromUtf8("detection_frame"))
        self.gridLayout_3.addWidget(self.detection_frame, 3, 1, 1, 1)
        self.vertical_layout_2.addWidget(self.object_detection_group)
        self.horizontal_layout_6 = QtGui.QHBoxLayout()
        self.horizontal_layout_6.setObjectName(_fromUtf8("horizontal_layout_6"))
        self.detection_color_label = QtGui.QLabel(self.central_widget)
        self.detection_color_label.setObjectName(_fromUtf8("detection_color_label"))
        self.horizontal_layout_6.addWidget(self.detection_color_label)
        self.detection_color_setting = QtGui.QComboBox(self.central_widget)
        self.detection_color_setting.setObjectName(_fromUtf8("detection_color_setting"))
        self.horizontal_layout_6.addWidget(self.detection_color_setting)
        self.vertical_layout_2.addLayout(self.horizontal_layout_6)
        self.horizontal_layout_7 = QtGui.QHBoxLayout()
        self.horizontal_layout_7.setObjectName(_fromUtf8("horizontal_layout_7"))
        self.selections_to_average_label = QtGui.QLabel(self.central_widget)
        self.selections_to_average_label.setObjectName(_fromUtf8("selections_to_average_label"))
        self.horizontal_layout_7.addWidget(self.selections_to_average_label)
        self.averaging_setting = QtGui.QSpinBox(self.central_widget)
        self.averaging_setting.setMinimum(1)
        self.averaging_setting.setMaximum(32)
        self.averaging_setting.setProperty("value", 2)
        self.averaging_setting.setObjectName(_fromUtf8("averaging_setting"))
        self.horizontal_layout_7.addWidget(self.averaging_setting)
        self.vertical_layout_2.addLayout(self.horizontal_layout_7)
        self.display_type_group = QtGui.QGroupBox(self.central_widget)
        self.display_type_group.setObjectName(_fromUtf8("display_type_group"))
        self.horizontalLayout_6 = QtGui.QHBoxLayout(self.display_type_group)
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.display_unfiltered_button = QtGui.QRadioButton(self.display_type_group)
        self.display_unfiltered_button.setChecked(True)
        self.display_unfiltered_button.setObjectName(_fromUtf8("display_unfiltered_button"))
        self.horizontalLayout_6.addWidget(self.display_unfiltered_button)
        self.display_reduced_button = QtGui.QRadioButton(self.display_type_group)
        self.display_reduced_button.setObjectName(_fromUtf8("display_reduced_button"))
        self.horizontalLayout_6.addWidget(self.display_reduced_button)
        self.display_extracted_button = QtGui.QRadioButton(self.display_type_group)
        self.display_extracted_button.setObjectName(_fromUtf8("display_extracted_button"))
        self.horizontalLayout_6.addWidget(self.display_extracted_button)
        self.vertical_layout_2.addWidget(self.display_type_group)
        self.color_ranges_group = QtGui.QGroupBox(self.central_widget)
        self.color_ranges_group.setObjectName(_fromUtf8("color_ranges_group"))
        self.gridLayout_2 = QtGui.QGridLayout(self.color_ranges_group)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.minimum_hsv_label = QtGui.QLabel(self.color_ranges_group)
        self.minimum_hsv_label.setObjectName(_fromUtf8("minimum_hsv_label"))
        self.gridLayout_2.addWidget(self.minimum_hsv_label, 0, 0, 1, 1)
        self.maximum_hsv_label = QtGui.QLabel(self.color_ranges_group)
        self.maximum_hsv_label.setObjectName(_fromUtf8("maximum_hsv_label"))
        self.gridLayout_2.addWidget(self.maximum_hsv_label, 1, 0, 1, 1)
        self.maximum_hsv_text = QtGui.QLineEdit(self.color_ranges_group)
        self.maximum_hsv_text.setObjectName(_fromUtf8("maximum_hsv_text"))
        self.gridLayout_2.addWidget(self.maximum_hsv_text, 1, 1, 1, 1)
        self.minimum_hsv_text = QtGui.QLineEdit(self.color_ranges_group)
        self.minimum_hsv_text.setObjectName(_fromUtf8("minimum_hsv_text"))
        self.gridLayout_2.addWidget(self.minimum_hsv_text, 0, 1, 1, 1)
        self.horizontal_layout_8 = QtGui.QHBoxLayout()
        self.horizontal_layout_8.setObjectName(_fromUtf8("horizontal_layout_8"))
        self.save_changes_button = QtGui.QPushButton(self.color_ranges_group)
        self.save_changes_button.setObjectName(_fromUtf8("save_changes_button"))
        self.horizontal_layout_8.addWidget(self.save_changes_button)
        self.undo_changes_button = QtGui.QPushButton(self.color_ranges_group)
        self.undo_changes_button.setObjectName(_fromUtf8("undo_changes_button"))
        self.horizontal_layout_8.addWidget(self.undo_changes_button)
        self.gridLayout_2.addLayout(self.horizontal_layout_8, 2, 1, 1, 1)
        self.vertical_layout_2.addWidget(self.color_ranges_group)
        self.logging_group = QtGui.QGroupBox(self.central_widget)
        self.logging_group.setObjectName(_fromUtf8("logging_group"))
        self.gridLayout_4 = QtGui.QGridLayout(self.logging_group)
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.log_output_box = QtGui.QPlainTextEdit(self.logging_group)
        self.log_output_box.setObjectName(_fromUtf8("log_output_box"))
        self.gridLayout_4.addWidget(self.log_output_box, 0, 0, 1, 1)
        self.vertical_layout_2.addWidget(self.logging_group)
        spacerItem9 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.vertical_layout_2.addItem(spacerItem9)
        self.horizontal_layout_1.addLayout(self.vertical_layout_2)
        self.horizontalLayout_3.addLayout(self.horizontal_layout_1)
        MainWindow.setCentralWidget(self.central_widget)
        self.menu_bar = QtGui.QMenuBar(MainWindow)
        self.menu_bar.setGeometry(QtCore.QRect(0, 0, 1010, 29))
        self.menu_bar.setObjectName(_fromUtf8("menu_bar"))
        self.menu_file = QtGui.QMenu(self.menu_bar)
        self.menu_file.setObjectName(_fromUtf8("menu_file"))
        MainWindow.setMenuBar(self.menu_bar)
        self.menu_new = QtGui.QAction(MainWindow)
        self.menu_new.setObjectName(_fromUtf8("menu_new"))
        self.menu_load = QtGui.QAction(MainWindow)
        self.menu_load.setObjectName(_fromUtf8("menu_load"))
        self.menu_save = QtGui.QAction(MainWindow)
        self.menu_save.setObjectName(_fromUtf8("menu_save"))
        self.menu_quit = QtGui.QAction(MainWindow)
        self.menu_quit.setObjectName(_fromUtf8("menu_quit"))
        self.menu_file.addAction(self.menu_new)
        self.menu_file.addAction(self.menu_load)
        self.menu_file.addAction(self.menu_save)
        self.menu_file.addSeparator()
        self.menu_file.addAction(self.menu_quit)
        self.menu_bar.addAction(self.menu_file.menuAction())

        self.retranslateUi(MainWindow)
        self.point_selector.setCurrentIndex(0)
        QtCore.QObject.connect(self.x1_slider, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x1_setting.setValue)
        QtCore.QObject.connect(self.y1_slider, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y1_setting.setValue)
        QtCore.QObject.connect(self.x1_setting, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x1_slider.setValue)
        QtCore.QObject.connect(self.y1_setting, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y1_slider.setValue)
        QtCore.QObject.connect(self.x2_slider, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x2_setting.setValue)
        QtCore.QObject.connect(self.y2_slider, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y2_setting.setValue)
        QtCore.QObject.connect(self.x2_setting, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.x2_slider.setValue)
        QtCore.QObject.connect(self.y2_setting, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.y2_slider.setValue)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.color_selection_group.setTitle(_translate("MainWindow", "Color Selection", None))
        self.selection_color_label.setText(_translate("MainWindow", "Selection Color:", None))
        self.new_color_name_setting.setText(_translate("MainWindow", "Input a name to generate a new calibration", None))
        self.new_color_button.setText(_translate("MainWindow", "New", None))
        self.delete_color_button.setText(_translate("MainWindow", "Delete", None))
        self.selection_managment_group.setTitle(_translate("MainWindow", "Selection Managment", None))
        self.x1_label.setText(_translate("MainWindow", "X Coordinate:", None))
        self.y1_label.setText(_translate("MainWindow", "Y Coordinate:", None))
        self.point_selector.setTabText(self.point_selector.indexOf(self.tab_1), _translate("MainWindow", "Point 1", None))
        self.y2_label.setText(_translate("MainWindow", "Y Coordinate:", None))
        self.x2_label.setText(_translate("MainWindow", "X Coordinate:", None))
        self.point_selector.setTabText(self.point_selector.indexOf(self.tab_11), _translate("MainWindow", "Point 2", None))
        self.overlap_prevention_group.setTitle(_translate("MainWindow", "Overlap Prevention", None))
        self.calibrate_button.setText(_translate("MainWindow", "Calibrate", None))
        self.object_detection_group.setTitle(_translate("MainWindow", "Object Detection", None))
        self.detection_color_label.setText(_translate("MainWindow", "Detection Color:", None))
        self.selections_to_average_label.setText(_translate("MainWindow", "Selections to Average:", None))
        self.display_type_group.setTitle(_translate("MainWindow", "Display Type", None))
        self.display_unfiltered_button.setText(_translate("MainWindow", "Unfiltered Input", None))
        self.display_reduced_button.setText(_translate("MainWindow", "Reduced Color", None))
        self.display_extracted_button.setText(_translate("MainWindow", "Extracted Color", None))
        self.color_ranges_group.setTitle(_translate("MainWindow", "Color HSV Ranges", None))
        self.minimum_hsv_label.setText(_translate("MainWindow", "Minimum:", None))
        self.maximum_hsv_label.setText(_translate("MainWindow", "Maximum:", None))
        self.save_changes_button.setText(_translate("MainWindow", "Save to Memory Only", None))
        self.undo_changes_button.setText(_translate("MainWindow", "Undo Changes", None))
        self.logging_group.setTitle(_translate("MainWindow", "Logging", None))
        self.menu_file.setTitle(_translate("MainWindow", "File", None))
        self.menu_new.setText(_translate("MainWindow", "New", None))
        self.menu_load.setText(_translate("MainWindow", "Load", None))
        self.menu_save.setText(_translate("MainWindow", "Save", None))
        self.menu_quit.setText(_translate("MainWindow", "Quit", None))

