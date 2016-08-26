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
    def __init__(self, parent_window):
        super(AdvancedOptions, self).__init__()

        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Advanced Options")

        self.parent = parent_window

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
        layout.addRow("# of Iterations", self.iterations)
        layout.addRow("# of Paths", self.tree_number)
        layout.addRow("Path Section Length", self.segment_length)
        layout.addRow("Method Type", self.method_box)
        layout.addRow("Start Apriltag #", self.start)
        layout.addRow("Goal Apriltag #", self.goal)
        layout.addRow("Low Enemy #", self.enemy_least)
        layout.addRow("High Enemy #", self.enemy_most)
        layout.addRow("Robot #", self.robot)

        parent_layout.addLayout(layout)
        parent_layout.addWidget(self.ok_btn)

        self.setLayout(parent_layout)

    def commitChanges(self):

        self.parent.setStartNumber(int(self.start.text()))
        self.parent.setGoalNumber(int(self.goal.text()))
        self.parent.setEnemyLowerBound(int(self.enemy_least.text()))
        self.parent.setEnemyUpperBound(int(self.enemy_most.text()))
        self.parent.setRobotNumber(int(self.robot.text()))

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
