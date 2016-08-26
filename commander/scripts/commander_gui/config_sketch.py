#!/usr/bin/env python

import sys, rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from tarrt_ros.msg import *
from tarrt_ros.srv import *

from sketch_window import Sketch

from error_popup.not_initialized import NotInitialized

from costmap_thread import CostmapThread
from tarrt_thread import TarrtThread

from advanced_options.tarrt_adv_options import TarrtAdvancedOptions

STARTX = 1000
STARTY = 1000
WIDTH = 250
HEIGHT = 200

class SketchConfig(QtGui.QMainWindow):

    def __init__(self):
        super(SketchConfig, self).__init__()
        rospy.init_node("sketch_config", anonymous=True)

        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Sketch Config")

        self.app_quit = QtGui.QAction("Quit", self)
        self.app_quit.triggered.connect(qApp.quit)
        self.app_quit.setShortcut("Ctrl+Q")

        self.adv_options = QtGui.QAction("Advanced Options", self)
        self.adv_options.triggered.connect(self.activateOptions)
        self.adv_options.setShortcut("Ctrl+A")

        self.options = TarrtAdvancedOptions(self)

        main_menu = self.menuBar()

        self.quick = QtGui.QCheckBox("", self)
        self.quick.stateChanged.connect(self.quickClicked)

        self.stealth = QtGui.QCheckBox("", self)
        self.stealth.stateChanged.connect(self.stealthClicked)

        self.safe = QtGui.QCheckBox("", self)
        self.safe.stateChanged.connect(self.safeClicked)

        load_menu = main_menu.addMenu("&File")
        load_menu.addAction(self.app_quit)
        load_menu.addAction(self.adv_options)

        self.launch_button = QtGui.QPushButton("Launch MORRF", self)
        self.launch_button.resize(250, 40)
        self.launch_button.move(0, 160)
        self.launch_button.setEnabled(False)
        self.launch_button.clicked.connect(self.launchMORRF)

        layout = QtGui.QVBoxLayout()
        inner_layout = QtGui.QFormLayout()

        inner_layout.addRow("Quickly", self.quick)
        inner_layout.addRow("Stealthy", self.stealth)
        inner_layout.addRow("Safely", self.safe)

        layout.addLayout(inner_layout)
        layout.addWidget(self.launch_button)

        self.win = QtGui.QWidget(self)
        self.win.setLayout(layout)

        self.setCentralWidget(self.win)

        self.image_window = Sketch()

        self.show()

    def activateOptions(self):
        self.options.activate()

    def setStartNumber(self, num):
        self.image_window.setStartNumber(num)

    def setGoalNumber(self, num):
        self.image_window.setGoalNumber(num)

    def setObstacleLowerBound(self, num):
        self.image_window.setObstacleLowerBound(num)

    def setObstacleUpperBound(self, num):
        self.image_window.setObstacleUpperBound(num)

    def setEnemyLowerBound(self, num):
        self.image_window.setEnemyLowerBound(num)

    def setEnemyUpperBound(self, num):
        self.image_window.setEnemyUpperBound(num)

    def setRobotNumber(self, num):
        self.image_window.setRobotNumber(num)

    def quickClicked(self):
        if self.quick.checkState() and self.stealth.checkState():
            self.stealth.setChecked(False)

        if self.quick.checkState() and self.safe.checkState():
            self.safe.setChecked(False)

        self.enableLaunch()

    def stealthClicked(self):
        if self.stealth.checkState() and self.quick.checkState():
            self.quick.setChecked(False)

        if self.stealth.checkState() and self.safe.checkState():
            self.safe.setChecked(False)

        self.enableLaunch()

    def safeClicked(self):
        if self.safe.checkState() and self.quick.checkState():
            self.quick.setChecked(False)

        if self.safe.checkState() and self.stealth.checkState():
            self.stealth.setChecked(False)

        self.enableLaunch()

    def enableLaunch(self):
        if self.quick.checkState() or self.stealth.checkState() or self.safe.checkState():
            self.launch_button.setEnabled(True)
        else:
            self.launch_button.setEnabled(False)

    def isCompleted(self):
        if not self.image_window.hasStart():
            return False

        elif not self.image_window.hasGoal():
            return False

        elif not (self.quick.checkState() or self.stealth.checkState() or self.safe.checkState()):
            return False

        else:
            return True

    def launchMORRF(self):
        if self.isCompleted():

            self.initializer = tarrt_ros.msg.tarrt_init()

            map_img = self.getMap()
            self.initializer.map = map_img

            safe = self.safe.isChecked()
            quick = self.quick.isChecked()
            stealth = self.stealth.isChecked()
            locs = self.image_window.getEnemyLocations()

            self.costmap_thread = CostmapThread(map_img, stealth, safe, locs)
            self.costmap_thread.start()

            self.connect(self.costmap_thread, QtCore.SIGNAL("COSTMAP_RESPONSE"), self.costmapCallback)

            goal = self.image_window.getGoalPoint()
            start = self.image_window.getStartPoint()

            self.initializer.goal.x = goal[0]
            self.initializer.goal.y = goal[1]

            self.initializer.start.x = start[0]
            self.initializer.start.y = start[1]

            self.initializer.width = map_img.width
            self.initializer.height = map_img.height

            self.initializer.number_of_iterations = self.options.getIterations()
            self.initializer.segment_length = self.options.getSegmentLength()
            #self.initializer.number_of_trees = self.options.getTreeNumber()

            print "quick " + str(quick)
            self.initializer.minimum_distance_enabled = quick

        else:
            self.error = NotInitialized()

    def costmapCallback(self, costmap_response):
        if len(costmap_response.cost_maps)>0:
            self.initializer.cost_map = costmap_response.cost_maps[0]

        self.tarrt_thread = TarrtThread(self.initializer)
        self.tarrt_thread.start()

        self.connect(self.tarrt_thread, QtCore.SIGNAL("TARRT_RESPONSE"), self.tarrtCallback)

    def tarrtCallback(self, tarrt_response):
        self.startPathCycler(tarrt_response)

    def startPathCycler(self, paths):
        self.image_window.startPathCycler(paths)

    def getMap(self):
        img = self.image_window.getImage()

        image = int16_image()
        image.width = img.width()
        image.height = img.height()

        for i in range(img.height()):
            for j in range(img.width()):
                c = QColor(img.pixel(j, i))

                image.int_array.append(c.blue())

        return image
