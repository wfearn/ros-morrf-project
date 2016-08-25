#!/usr/bin/env python

import sys, rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

STARTX = 1200
STARTY = 1200
WIDTH = 200
HEIGHT = 400

class AdvancedOptions(QtGui.QWidget):
    def __init__(self):
        super(AdvancedOptions, self).__init__()

        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Advanced Options")

        self.iterations = QtGui.QLineEdit()
        self.iterations.setFrame(True)
        self.iterations.setMaxLength(4)
        self.iterations.setText("2000")

        self.tree_number = QtGui.QLineEdit()
        self.tree_number.setFrame(True)
        self.tree_number.setMaxLength(4)
        self.tree_number.setText("5")

        self.segment_length = QtGui.QLineEdit()
        self.segment_length.setFrame(True)
        self.segment_length.setMaxLength(4)
        self.segment_length.setText("5")

        self.method_box = QtGui.QComboBox()
        self.method_box.addItem("Weighted Sum")
        self.method_box.addItem("Tchebycheff")
        self.method_box.addItem("Boundary Intersection")

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

        layout = QtGui.QFormLayout()
        layout.addRow("# of Iterations", self.iterations)
        layout.addRow("# of Paths", self.tree_number)
        layout.addRow("Path Section Length", self.segment_length)
        layout.addRow("Method Type", self.method_box)
        layout.addRow("Start Apriltag #", self.start)
        layout.addRow("Goal Apriltag #", self.goal)
        layout.addRow("Low Obstacle #", self.obs_least)
        layout.addRow("High Obstacle #", self.obs_most)
        layout.addRow("Low Enemy #", self.enemy_least)
        layout.addRow("High Enemy #", self.enemy_most)

        self.setLayout(layout)

    def getIterations(self):
        return int(self.iterations.text())

    def getTreeNumber(self):
        return int(self.tree_number.text())

    def getSegmentLength(self):
        return int(self.segment_length.text())

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

    def getMethodNumber(self):
        if self.method_box.currentText() == "Weighted Sum":
            return 0
        elif self.method_box.currentText() == "Tchebycheff":
            return 1
        else:
            return 2

    def activate(self):
        self.show()
