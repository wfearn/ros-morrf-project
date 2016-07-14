#!/usr/bin/env python

import sys
import rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from image_window import Image
from image_window import Image

from morrf_ros.msg import *
import cv2
import numpy as np
import json

from error_popup.not_initialized import NotInitialized
from error_popup.no_image_selected import NoImage

from publishers.image_publisher import StartImagePublisher
from publishers.commander_publisher import StartCommanderPublisher
from publishers.costmap_publisher import StartCostmapPublisher
from publishers.continue_publisher import StartContinuePublisher

STARTX = 1000
STARTY = 1000
WIDTH = 250
HEIGHT = 700

MORRF_OUTPUT_FILE = "/home/tkatuka/Dropbox/MORRF_OUTPUT/morrf_output/{}"
IMG_OUTPUT_FILE = "/home/tkatuka/Dropbox/MORRF_OUTPUT/maps/{}"

PATHS_FILE = MORRF_OUTPUT_FILE.format("paths.txt")
COSTS_FILE = MORRF_OUTPUT_FILE.format("costs.txt")
SAFE_VALS_FILE = MORRF_OUTPUT_FILE.format("safe_values.txt")
STEALTH_VALS_FILE = MORRF_OUTPUT_FILE.format("stealth_values.txt")
ENEMIES_FILE = MORRF_OUTPUT_FILE.format("enemies.txt")

MAP_IMG = IMG_OUTPUT_FILE.format("map.png")
BOUNDARY_IMG = IMG_OUTPUT_FILE.format("boundary.png")

JSON_FILE = "/home/tkatuka/Dropbox/MORRF_OUTPUT/morrf.json"



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

	self.statusBar()

        launch_button = QtGui.QPushButton("Launch MORRF", self)
        launch_button.resize(250, 50)
        launch_button.clicked.connect(self.launch_morrf)

        self.pick_paths = QtGui.QPushButton("Pick path", self)
        self.pick_paths.resize(250, 50)
        self.pick_paths.clicked.connect(self.sendToPathPicker)
        self.pick_paths.setEnabled(False)

        self.continue_btn = QtGui.QPushButton("Continue MORRF", self)
        self.continue_btn.resize(250, 50)
        self.continue_btn.clicked.connect(self.continueMorrf)
	self.continue_btn.setStatusTip('To make paths smoother')
        self.continue_btn.setEnabled(False)

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
        layout.addRow("Stealthy", self.stealth)
        layout.addRow("Safely", self.safe)
        layout.addRow("Quickly", self.min_distance)
        layout.addRow("Method Type", self.method_box)

        main_layout = QtGui.QVBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addWidget(launch_button)
        main_layout.addWidget(self.pick_paths)
        main_layout.addWidget(self.continue_btn)

        self.win = QtGui.QWidget(self)
        self.win.setLayout(main_layout)

        self.setCentralWidget(self.win)
        self.show()

    def launch_morrf(self):
        if not hasattr(self, "image_window"):
            self.error = NoImage()

        elif self.is_completed() and self.image_window.isCompleted():

	    print "Launching MORRF"

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

            self.costmap_response = StartCostmapPublisher(initializer.map, self.stealth.isChecked(), self.safe.isChecked(), self.image_window.getEnemyLocations())

            initializer.cost_maps = self.costmap_response.cost_maps

            self.morrf_response = StartCommanderPublisher(initializer)
	    
            print "MORRF paths received, printing..."

            self.image_window.startPathCycler(self.morrf_response)
            self.pick_paths.setEnabled(True)
            self.continue_btn.setEnabled(True)

        else:
            self.error = NotInitialized()

    def sendToPathPicker(self):
        print "Sending to path palette..."

        self.outputToDropbox(self.morrf_response)

    def saveBoundImage(self, image):

        test = QImage(image.width, image.height, QImage.Format_RGB16)

        for i in range(image.height):
            for j in range(image.width):

                index = i * image.width + j
                color = image.int_array[index]

                qrgb = QColor(color, color, color)

                test.setPixel(j, i, qrgb.rgb())

        test.save(BOUNDARY_IMG)

    def continueMorrf(self):
        self.morrf_response = StartContinuePublisher(int(self.iterations.text()))
        self.image_window.startPathCycler(self.morrf_response)

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
            image = int16_image()

            img = cv2.imread(str(self.image_name))

            image.width = img.shape[1]
            image.height = img.shape[0]
            image.name = str(self.image_name)

            for j in range(image.height):
                for i in range(image.width):
                    image.int_array.append(np.int16(img[j,i,0]))

            return image
        else:
            return None


    def contextMenuRequested(self, point):
        pass

    def outputToDropbox(self, morrf_output):

        self.image_window.saveMapToDropbox()
        self.saveBoundImage(self.costmap_response.boundary_image)

        start = self.image_window.getStartPoint()
        goal = self.image_window.getGoalPoint()
        paths = self.writePathsToFile(morrf_output.paths)

        json_output = {}

        json_output["start"] = str(start[0]) + "," + str(start[1])
        json_output["goal"] = str(goal[0]) + "," + str(goal[1])
        json_output["map_image"] = self.image_window.getMapName()
        #json_output["enemies"] = self.writeEnemyPosToFile(self.image_window.getEnemyLocations())
	json_output["enemies"] = self.writeEnemyPos(self.image_window.getEnemyLocations())	
        json_output["boundary_image"] = BOUNDARY_IMG
        json_output["paths"] = paths[0]
        json_output["costs"] = paths[1]

        self.writeCostValuesToFile(self.costmap_response.cost_values, json_output)

        js = open(JSON_FILE, "w")
        js.write(json.dumps(json_output))
        js.close()

    def writeCostValuesToFile(self, cost_values, json_file):
        if len(cost_values) == 2:
            for i in range(len(cost_values)):
                cv = cost_values[i]

                if cv.name == "Stealth":
                    f = open(STEALTH_VALS_FILE, "w")

                    for j in range(len(cv.vals)):
                        f.write("%s %s %s\n" % (cv.vals[j].position.x, cv.vals[j].position.y, cv.vals[j].cost))


                    json_file["stealth_vals"] = f.name
                    f.close()

                else:
                    f = open(SAFE_VALS_FILE, "w")

                    for j in range(len(cv.vals)):
                        f.write("%s %s %s\n" % (cv.vals[j].position.x, cv.vals[j].position.y, cv.vals[j].cost))

                    json_file["safe_vals"] = f.name
                    f.close()

        else:
            cv = cost_values[0]

            if cv.name == "Stealth":
                f = open(STEALTH_VALS_FILE, "w")

                for j in range(len(cv.vals)):
                    f.write("%s %s %s\n" % (cv.vals[j].position.x, cv.vals[j].position.y, cv.vals[j].cost))

                json_file["stealth_vals"] = f.name
                f.close()
            else:
                f = open(SAFE_VALS_FILE, "w")

                for j in range(len(cv.vals)):
                    f.write("%s %s %s\n" % (cv.vals[j].position.x, cv.vals[j].position.y, cv.vals[j].cost))

                json_file["safe_vals"] = f.name
                f.close()

    def writeEnemyPos(self, enemy_locations):
        #enemies = open(ENEMIES_FILE, "w")
	
	toReturn = ""

        for pos in enemy_locations:
            toReturn += (str(pos.x) + "," + str(pos.y) + "E")

        return toReturn

    def writePathsToFile(self, paths):

        waypoint_output = open(PATHS_FILE, "w")
        cost_output = open(COSTS_FILE, "w")

        for i in range(len(paths)):
            for point in paths[i].waypoints:
                waypoint_output.write( str(int(point.x)) + " " + str(int(point.y)) + "\t" )

            waypoint_output.write("\n")

            if len(paths[i].cost) == 3:
                for cost in paths[i].cost:
                    cost_output.write(str( round(cost.data, 3) ) + "\t")

                cost_output.write("\n")

            elif len(paths[i].cost) == 2:
                if self.min_distance.isChecked():
                    cost_output.write(str( round(paths[i].cost[0].data, 3) + "\t") )

                    if self.stealth.isChecked():
                        cost_output.write(str(paths[i].cost[1].data) + " ")
                        cost_output.write(str(0))

                    else:
                        cost_output.write(str(0) + " ")
                        cost_output.write(str(paths[i].cost[1].data))

                elif self.stealth.isChecked():
                    cost_output.write(str(0) + " ")
                    for cost in paths[i].cost:
                        cost_output.write(str(cost.data) + " ")

                else:
                    cost_output.write(str(0) + " " + str(0) + " ")
                    cost_output.write(str(paths[i].cost[0].data))

                cost_output.write("\n")

            else:
                if self.min_distance.isChecked():
                    cost_output.write(str(paths[i].cost[0].data) + " ")
                    cost_output.write(str(0) + " " + str(0))

                elif self.stealth.isChecked():
                    cost_output.write(str(0) + " ")
                    cost_output.write(str(paths[i].cost[0].data) + " ")
                    cost_output.write(str(0))

                else:
                    cost_output.write(str(0) + " " + str(0) + " ")
                    cost_output.write(str(paths[i].cost[0].data) + " ")

                cost_output.write("\n")

        waypoint_name = waypoint_output.name
        cost_name = cost_output.name

        waypoint_output.close()
        cost_output.close()


        return (waypoint_name, cost_name)
