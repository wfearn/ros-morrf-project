#!/usr/bin/env python
import sys, rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from std_msgs.msg import *

class MORRFProgressBar(QWidget):
    def __init__(self):
        QWidget.__init__(self)

        self.setGeometry(800, 1200, 400, 50)
        self.setWindowTitle("MORRF Progress Bar")

        self.progress = QtGui.QProgressBar(self)
        self.progress.setGeometry(50, 10, 300, 30)
        self.progress.setTextVisible(True)

        self.morrf_progress = rospy.Subscriber("/morrf_status", Float64, self.updateProgress)

    def getValue(self):
        return self.progress.value()

    def activate(self):
        self.show()

    def updateProgress(self, data):
        if data.data >= 100:
            self.hide()

        else:
            self.progress.setValue(data.data)
