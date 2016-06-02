import sys
import rospy
from PyQt4 import QtGui, QtCore
from image_window import Image
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from image_window import Image
from morrf_ros.msg import *
import cv2
import numpy as np
from commander_publisher import StartCommanderPublisher

STARTX = 1000
STARTY = 1000
WIDTH = 250
HEIGHT = 700

class Config(QtGui.QMainWindow):

    def __init__(self):
        super(Config, self).__init__()
        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Config")

        self.image_load = QtGui.QAction("Load Image", self)
        self.image_load.triggered.connect(self.load_image)

        main_menu = self.menuBar()

        load_menu = main_menu.addMenu('&File')
        load_menu.addAction(self.image_load)


        launch_button = QtGui.QPushButton("Launch MORRF", self)
        launch_button.resize(250, 50)
        launch_button.clicked.connect(self.launch_morrf)

        self.iterations = QtGui.QLineEdit()
        self.iterations.setFrame(True)
        self.iterations.setMaxLength(4)

        self.tree_number = QtGui.QLineEdit()
        self.tree_number.setFrame(True)
        self.tree_number.setMaxLength(4)

        self.segment_length = QtGui.QLineEdit()
        self.segment_length.setFrame(True)
        self.segment_length.setMaxLength(4)

        self.objective_number = QtGui.QLineEdit()
        self.objective_number.setFrame(True)
        self.objective_number.setMaxLength(4)

        self.method_type = QtGui.QLineEdit()
        self.method_type.setFrame(True)
        self.method_type.setMaxLength(1)

        self.min_distance = QtGui.QCheckBox("", self)

        layout = QtGui.QFormLayout()
        layout.addRow("# Of Iterations", self.iterations)
        layout.addRow("# Of Trees", self.tree_number)
        layout.addRow("Segment Length", self.segment_length)
        layout.addRow("# Of Objectives", self.objective_number)
        layout.addRow("Minimize Distance", self.min_distance)
        layout.addRow("Method Type", self.method_type)

        main_layout = QtGui.QVBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addWidget(launch_button)

        self.win = QtGui.QWidget(self)
        self.win.setLayout(main_layout)

        self.setCentralWidget(self.win)
        self.show()

    def launch_morrf(self):
        if not hasattr(self, "image_window"):
            print "Morrf parameters not completed, need to initialize image"

        elif self.is_completed() and self.image_window.isCompleted():
            initializer = morrf_init()

            goal = self.image_window.getGoalPoint()
            start = self.image_window.getStartPoint()

            initializer.goal.x = goal[0]
            initializer.goal.y = goal[1]
            initializer.start.x = start[0]
            initializer.start.y = start[1]
            initializer.number_of_iterations = int(self.iterations.text())
            initializer.segment_length = int(self.segment_length.text())
            initializer.number_of_trees = int(self.tree_number.text())
            initializer.objective_number = int(self.objective_number.text())
            initializer.minimum_distance_enabled = self.min_distance.isChecked()
            initializer.map = self.map_convert()

            print "Goal is: %s, %s" % (initializer.goal.x, initializer.goal.y)
            print "Start is: %s, %s" % (initializer.start.x, initializer.start.y)
            print "Iterations are: %s" % (initializer.number_of_iterations)
            print "Segment Length is: %s" % (initializer.segment_length)
            print "Number of trees is: %s" % (initializer.number_of_trees)
            print "Objective Number is: %s" % (initializer.objective_number)
            print "Minimum distance enabled is: %s" % (initializer.minimum_distance_enabled)
            print "Map image name is: %s" % initializer.map.name
            print "Map height is: %s" % initializer.map.height
            print "Map width is: %s" % initializer.map.width
            #print "Map pixel values are %s" % initializer.map.int_array

            StartCommanderPublisher(initializer)

        else:
            print "Morrf parameters not completed, initialize config or image parameters"

    def is_completed(self):
        if self.iterations.text() == "":
            return False
        elif self.tree_number.text() == "":
            return False
        elif self.segment_length.text() == "":
            return False
        elif self.objective_number.text() == "":
            return False
        else:
            return True

    def load_image(self):
        self.image_name = QtGui.QFileDialog.getOpenFileName(self, "Load Image")
        self.image_window = Image(self.image_name)

    def map_convert(self):
        if hasattr(self, "image_name"):
            image = int8_image()

            img = cv2.imread(str(self.image_name))

            image.width = img.shape[0]
            image.height = img.shape[1]
            image.name = str(self.image_name)

            for i in range(image.width):
                for j in range(image.height):
                    image.int_array.append(int(img[i,j,0]))

            return image
        else:
            return None
