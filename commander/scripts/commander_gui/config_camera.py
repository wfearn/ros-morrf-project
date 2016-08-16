#!/usr/bin/env python

import sys, rospy, json, threading

from pprint import pprint

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from image_window import Image

from morrf_ros.msg import *

from commander.msg import *

import cv2
import numpy as np
import math

from camera_window import CameraWindow

from objective.objective import Objective

from error_popup.not_initialized import NotInitialized
from error_popup.no_image_selected import NoImage
from error_popup.no_path_error import NoPath

from publishers.costmap_publisher import StartCostmapPublisher
from publishers.commander_publisher import StartCommanderPublisher
from publishers.follow_publisher import StartFollowPublisher
from publishers.continue_publisher import StartContinuePublisher

from advanced_options.advanced_options import AdvancedOptions

from progress_bar.progress_bar import MORRFProgressBar

from commander_thread import CommanderThread
from costmap_thread import CostmapThread
from continue_thread import ContinueThread

STARTX = 1200
STARTY = 1200
WIDTH = 200
HEIGHT = 300
ROBOT_PATH = "{}/data/objective_icons/robot.png"


class CameraConfig(QtGui.QMainWindow):
    def __init__(self):
        super(CameraConfig, self).__init__()
        rospy.init_node("camera_config", anonymous=True)

        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Camera Config")
        self.center()

        self.image_window = CameraWindow()

        self.progress_bar = MORRFProgressBar()

        action = QtGui.QAction("Close", self)
        action.setShortcut("Ctrl+C")
        action.triggered.connect(QtCore.QCoreApplication.instance().quit)

        self.addAction(action)

        self.statusBar()

        self.view()

        self.adv_options = QtGui.QAction("Advanced Options", self)
        self.adv_options.triggered.connect(self.activateOptions)

        self.appQuit = QtGui.QAction("Quit", self)
        self.appQuit.triggered.connect(qApp.quit)
        self.appQuit.setShortcut("Ctrl+Q")

        main_menu = self.menuBar()

        file_menu = main_menu.addMenu("&File")
        file_menu.addAction(self.adv_options)
        file_menu.addAction(self.appQuit)

        self.options = AdvancedOptions()


    def view(self):
        btn = QtGui.QPushButton("Exit", self)
        btn.clicked.connect(QtCore.QCoreApplication.instance().quit)

        self.robot = QtGui.QPushButton("Send Robot", self)
        self.robot.setEnabled(False)
        self.robot.clicked.connect(self.sendToRobot)

        self.quick = QtGui.QCheckBox("Quickly", self)
        self.quick.setChecked(False)
        self.quick.stateChanged.connect(self.enableLaunch)

        self.stealth = QtGui.QCheckBox("Stealthily", self)
        self.stealth.setChecked(False)
        self.stealth.stateChanged.connect(self.enableLaunch)

        self.safe = QtGui.QCheckBox("Safely", self)
        self.safe.setChecked(False)
        self.safe.stateChanged.connect(self.enableLaunch)

        self.launch = QtGui.QPushButton("Launch MORRF", self)
        self.launch.setEnabled(False)
        self.launch.clicked.connect(self.launchMORRF)

        self.send = QtGui.QPushButton("Pick Path", self)
        self.send.setEnabled(False)
        self.send.clicked.connect(self.sendToPathPicker)

        self.continue_btn = QtGui.QPushButton("Continue MORRF", self)
        self.continue_btn.resize(250, 50)
        self.continue_btn.clicked.connect(self.continueMORRF)
        self.continue_btn.setEnabled(False)

        layout = QVBoxLayout()
        layout.addWidget(self.quick)
        layout.addWidget(self.stealth)
        layout.addWidget(self.safe)
        layout.addWidget(self.launch)
        layout.addWidget(self.robot)
        layout.addWidget(self.send)
        layout.addWidget(self.continue_btn)
        layout.addWidget(btn)

        self.widget = QtGui.QWidget(self)
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        self.show()

    def enableLaunch(self, state):
        if state == QtCore.Qt.Checked:
            self.launch.setEnabled(True)

        elif self.quick.checkState() or self.stealth.checkState() or self.safe.checkState():
            self.launch.setEnabled(True)

        else:
            self.launch.setEnabled(False)

    def isCompleted(self):
        return self.image_window.isInitialized()

    def continueMORRF(self):
        self.progress_bar.activate()

        #self.morrf_response = StartContinuePublisher(self.options.getIterations())

        self.continue_thread = ContinueThread(self.options.getIterations())
        self.continue_thread.start()

        self.connect(self.continue_thread, QtCore.SIGNAL("CONTINUE_RESPONSE"), self.continueCallback)
        #self.image_window.startPathCycler(self.morrf_response)

    def continueCallback(self, morrf_response):
        self.image_window.startPathCycler(morrf_response)


    def activateOptions(self):
        self.options.activate()

    def sendToRobot(self):
        StartFollowPublisher(self.image_window.getMORRFPath())

    def sendToPathPicker(self):
        print "Sending to path palette..."

        self.image_window.saveMapToDropbox()
        self.saveBoundImage(self.costmap_response.boundary_image)

    def center(self):
        frameGm = self.frameGeometry()
        centerPoint = QtGui.QDesktopWidget().availableGeometry().center()
        frameGm.moveCenter(centerPoint)
        self.move(frameGm.topLeft())

    def launchMORRF(self):
        if self.isCompleted():
            self.progress_bar.activate()

            self.initialize()

        else:
            self.error = NotInitialized()

    def costmapCallback(self, costmap_response):
        self.initializer.cost_maps = costmap_response.cost_maps

        self.morrf_thread = CommanderThread(self.initializer)
        self.morrf_thread.start()

        self.connect(self.morrf_thread, QtCore.SIGNAL("MORRF_RESPONSE"), self.MORRFCallback)


    def MORRFCallback(self, morrf_response):

        self.startPathCycler(morrf_response)

    def startPathCycler(self, paths):
        self.image_window.startPathCycler(paths)

        if hasattr(self.image_window, 'morrf_paths'):
            self.robot.setEnabled(True)
            self.continue_btn.setEnabled(True)
            self.send.setEnabled(True)

        else:
            self.robot.setEnabled(False)
            self.continue_btn.setEnabled(False)
            self.send.setEnabled(False)

            self.error = NoPath()


    def initialize(self):
        print "Launching MORRF..."

        self.initializer = morrf_ros.msg.morrf_init()

        map_img = self.mapConvert()
        self.initializer.map = map_img

        safe = self.safe.isChecked()
        quick = self.quick.isChecked()
        stealth = self.stealth.isChecked()
        locs = self.image_window.getEnemyLocations()

        print "Sending map to costmap generator..."

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
        self.initializer.number_of_trees = self.options.getTreeNumber()

        self.initializer.objective_number = safe + quick + stealth
        self.initializer.minimum_distance_enabled = quick
        self.initializer.method_type = self.options.getMethodNumber()

    def mapConvert(self):
        img = self.image_window.getMORRFImage()
        img.save("/home/wfearn/Documents/camera_img.png")

        image = int16_image()
        image.width = img.width()
        image.height = img.height()

        for i in range(img.height()):
            for j in range(img.width()):
                c = QColor(img.pixel(j, i))

                image.int_array.append(c.blue())

        return image

