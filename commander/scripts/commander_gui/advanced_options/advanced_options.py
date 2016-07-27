#!/usr/bin/env python

import sys, rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

STARTX = 1200
STARTY = 1200
WIDTH = 200
HEIGHT = 200

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

        layout = QtGui.QFormLayout()
        layout.addRow("# of Iterations", self.iterations)
        layout.addRow("# of Paths", self.tree_number)
        layout.addRow("Path Section Length", self.segment_length)
        layout.addRow("Method Type", self.method_box)

        self.setLayout(layout)

    def getIterations(self):
        return int(self.iterations.text())

    def getTreeNumber(self):
        return int(self.tree_number.text())

    def getSegmentLength(self):
        return int(self.segment_length.text())

    def getMethodNumber(self):
        if self.method_box.currentText() == "Weighted Sum":
            return 0
        elif self.method_box.currentText() == "Tchebycheff":
            return 1
        else:
            return 2

    def activate(self):
        self.show()
