#!/usr/bin/env python

import sys, rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

STARTX = 1200
STARTY = 1200
WIDTH = 250
HEIGHT = 450

class TarrtAdvancedOptions(QtGui.QWidget):
    def __init__(self, parent_window):
        super(TarrtAdvancedOptions, self).__init__()

        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Tarrt Advanced Options")

        self.parent = parent_window

        self.iterations = QtGui.QLineEdit()
        self.iterations.setFrame(True)
        self.iterations.setMaxLength(5)
        self.iterations.setText("2000")

        self.segment_length = QtGui.QLineEdit()
        self.segment_length.setFrame(True)
        self.segment_length.setMaxLength(3)
        self.segment_length.setText("5")

        self.number_of_paths = QtGui.QLineEdit()
        self.number_of_paths.setFrame(True)
        self.number_of_paths.setMaxLength(3)
        self.number_of_paths.setText("5")

        self.start = QtGui.QLineEdit()
        self.start.setFrame(True)
        self.start.setMaxLength(2)
        self.start.setText("50")

        self.goal = QtGui.QLineEdit()
        self.goal.setFrame(True)
        self.goal.setMaxLength(2)
        self.goal.setText("51")

        self.obs_least = QtGui.QLineEdit()
        self.obs_least.setFrame(True)
        self.obs_least.setMaxLength(2)
        self.obs_least.setText("30")

        self.obs_most = QtGui.QLineEdit()
        self.obs_most.setFrame(True)
        self.obs_most.setMaxLength(2)
        self.obs_most.setText("39")

        self.enemy_least = QtGui.QLineEdit()
        self.enemy_least.setFrame(True)
        self.enemy_least.setMaxLength(2)
        self.enemy_least.setText("40")

        self.enemy_most = QtGui.QLineEdit()
        self.enemy_most.setFrame(True)
        self.enemy_most.setMaxLength(2)
        self.enemy_most.setText("49")

        self.robot = QtGui.QLineEdit()
        self.robot.setFrame(True)
        self.robot.setMaxLength(2)
        self.robot.setText("97")

        self.ok_btn = QtGui.QPushButton("Commit Changes", self)
        self.ok_btn.resize(70, 40)
        self.ok_btn.clicked.connect(self.commitChanges)

        parent_layout = QtGui.QVBoxLayout()

        layout = QtGui.QFormLayout()
        layout.addRow("# Of Iterations", self.iterations)
        layout.addRow("# Of Paths", self.number_of_paths)
        layout.addRow("Segment Length", self.segment_length)
        layout.addRow("Start Apriltag #", self.start)
        layout.addRow("Goal Apriltag #", self.goal)
        layout.addRow("Low Obstacle #", self.obs_least)
        layout.addRow("High Obstacle #", self.obs_most)
        layout.addRow("Low Enemy #", self.enemy_least)
        layout.addRow("High Enemy #", self.enemy_most)
        layout.addRow("Robot #", self.robot)

        parent_layout.addLayout(layout)
        parent_layout.addWidget(self.ok_btn)

        self.setLayout(parent_layout)

    def commitChanges(self):

        self.parent.setStartNumber(int(self.start.text()))
        self.parent.setGoalNumber(int(self.goal.text()))
        self.parent.setObstacleLowerBound(int(self.obs_least.text()))
        self.parent.setObstacleUpperBound(int(self.obs_most.text()))
        self.parent.setEnemyLowerBound(int(self.enemy_least.text()))
        self.parent.setEnemyUpperBound(int(self.enemy_most.text()))
        self.parent.setRobotNumber(int(self.robot.text()))

    def getIterations(self):
        return int(self.iterations.text())

    def getNumberOfPaths(self):
        return int(self.number_of_paths.text())

    def getSegmentLength(self):
        return int(self.segment_length.text())

    def getRobotNumber(self):
        return int(self.robot.text())

    def getIterations(self):
        return 3000 #int(self.iterations.text())

    def getSegmentLength(self):
        return 4

    def getStartNumber(self):
        return int(self.start.text())

    def getGoalNumber(self):
        return int(self.goal.text())

    def getObstacleLowerBound(self):
        return int(self.obs_least.text())

    def getObstacleUpperBound(self):
        return int(self.obs_most.text())

    def getEnemyLowerBound(self):
        return int(self.enemy_least.text())

    def getEnemyUpperBound(self):
        return int(self.enemy_most.text())

    def activate(self):
        self.show()
