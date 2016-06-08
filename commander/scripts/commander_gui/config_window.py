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
import json

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
        self.appQuit = QtGui.QAction("Quit", self)
        self.appQuit.triggered.connect(qApp.quit)

        main_menu = self.menuBar()

        load_menu = main_menu.addMenu('&File')
        load_menu.addAction(self.image_load)
        load_menu.addAction(self.appQuit)


        launch_button = QtGui.QPushButton("Launch MORRF", self)
        launch_button.resize(250, 50)
        launch_button.clicked.connect(self.launch_morrf)

        self.iterations = QtGui.QLineEdit()
        self.iterations.setFrame(True)
        self.iterations.setMaxLength(4)
        self.iterations.setText("500")

        self.tree_number = QtGui.QLineEdit()
        self.tree_number.setFrame(True)
        self.tree_number.setMaxLength(4)
        self.tree_number.setText("5")

        self.segment_length = QtGui.QLineEdit()
        self.segment_length.setFrame(True)
        self.segment_length.setMaxLength(4)
        self.segment_length.setText("5")

        self.objective_number = QtGui.QLineEdit()
        self.objective_number.setFrame(True)
        self.objective_number.setMaxLength(4)
        self.objective_number.setText("1")

        self.method_box = QtGui.QComboBox()
        self.method_box.addItem("Weighted Sum")
        self.method_box.addItem("Tchebycheff")
        self.method_box.addItem("Boundary Intersection")

        self.min_distance = QtGui.QCheckBox("", self)
        self.min_distance.setChecked(True)

        self.stealth = QtGui.QCheckBox("", self)
        self.stealth.setChecked(False)

        self.safe = QtGui.QCheckBox("", self)
        self.safe.setChecked(False)

        layout = QtGui.QFormLayout()
        layout.addRow("# Of Iterations", self.iterations)
        layout.addRow("# Of Trees", self.tree_number)
        layout.addRow("Segment Length", self.segment_length)
        #layout.addRow("# Of Objectives", self.objective_number)
        layout.addRow("Stealthy", self.stealth)
        layout.addRow("Safely", self.safe)
        layout.addRow("Quickly", self.min_distance)
        layout.addRow("Method Type", self.method_box)

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

            #Deinitializing variable to prevent errors
            self.image_window.delMorrfPaths()

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
            initializer.objective_number = int(self.getObjectiveNumbers())
            initializer.minimum_distance_enabled = self.min_distance.isChecked()
            initializer.method_type = int(self.getMethodType())
            initializer.map = self.map_convert()
            initializer.width = initializer.map.width
            initializer.height = initializer.map.height

            print "Goal is: %s, %s" % (initializer.goal.x, initializer.goal.y)
            print "Start is: %s, %s" % (initializer.start.x, initializer.start.y)
            print "Iterations are: %s" % (initializer.number_of_iterations)
            print "Segment Length is: %s" % (initializer.segment_length)
            print "Number of trees is: %s" % (initializer.number_of_trees)
            print "Objective Number is: %s" % (initializer.objective_number)
            print "Minimum distance enabled is: %s" % (initializer.minimum_distance_enabled)
            print "Method type is: %s" % self.method_box.currentText()
            print "Map image name is: %s" % initializer.map.name
            print "Map height is: %s" % initializer.map.height
            print "Map width is: %s" % initializer.map.width
            #print "Map pixel values are %s" % initializer.map.int_array

            response = StartCommanderPublisher(initializer)

            self.image_window.printMorrfPaths(response)

            #self.outputToDropbox(response)

        else:
            print "Morrf parameters not completed, initialize config or image parameters"

    def getObjectiveNumbers(self):

        counter = 0

        if self.stealth.isChecked():
            counter +=1

        if self.safe.isChecked():
            counter += 1

        if self.min_distance.isChecked():
            counter +=1

        return counter

    def getMethodType(self):
        if self.method_box.currentText() == "Weighted Sum":
            return 0
        elif self.method_box.currentText() == "Tchebycheff":
            return 1
        else:
            return 2

   # def setMethodType(self, text):
   #     if text == "Weighted Sum":
   #         self.method_type = 0
   #     elif text == "Tchebycheff":
   #         self.method_type = 1
   #     else:
   #         self.method_type = 2

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

            image.width = img.shape[1]
            print "Image width is %s" % image.width
            image.height = img.shape[0]
            print "Image height is %s" % image.height
            image.name = str(self.image_name)

            for j in range(image.width):
                for i in range(image.height): 
                    image.int_array.append(np.int16(img[i,j,0]))

            return image
        else:
            return None


    def contextMenuRequested(self, point):
        pass

    def outputToDropbox(self, morrf_output):

        self.image_window.saveMapToDropbox()

        morrf_output_path = "/home/wfearn/Dropbox/morrf_output/morrf_output.txt"
        map_output_path = "/home/wfearn/Dropbox/map/%s" % self.image_window.getMapName()
        start = self.image_window.getStartPoint()
        goal = self.image_window.getGoalPoint()

        json_output = {}

        json_output["enemies"] = []
        for pos in self.image_window.getEnemyLocations():
            json_output["enemies"].append(str(pos[0]) + "," + str(pos[1]))

        json_output["start"] = str(start[0]) + "," + str(start[1])
        json_output["goal"] = str(goal[0]) + "," + str(goal[1])

        #Making file path concise for retrieval from other computer
        path_array = morrf_output_path.split("/")
        json_output["morrf_output"] = path_array[4] + "/" + path_array[5]

        waypoint_output = []
        cost_output = []

        for index in range(len(morrf_output.paths)):
            waypoint_output.append([])
            cost_output.append([])

            for point in morrf_output.paths[index].waypoints:
                waypoint_output[index].append(point)

            for cost in morrf_output.paths[index].cost:
                cost_output[index].append(cost)

        print waypoint_output
        print cost_output
        print json.dumps(json_output)

        f = open(morrf_output_path, "w")

        for path in cost_output:
            for cost in path:
                f.write(str(cost.data) + " ")

            f.write("\n")

        for path in waypoint_output:
            for point in path:
                f.write(str(point.x) + " " + str(point.y) + " ")
            f.write("\n")

        f.close()
