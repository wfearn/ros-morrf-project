#!/usr/bin/env python
import sys, rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from std_msgs.msg import *

from progress_subscriber_thread import ProgressBarThread

class MORRFProgressBar(QWidget):
    def __init__(self):
        QWidget.__init__(self)

        self.setGeometry(800, 1200, 400, 50)
        self.setWindowTitle("MORRF Progress Bar")

        self.progress = QtGui.QProgressBar(self)
        self.progress.setGeometry(50, 10, 300, 30)
        self.progress.setTextVisible(True)

        self.morrf_progress = ProgressBarThread()

    def getValue(self):
        return self.progress.value()

    def activate(self):
        self.progress.setValue(0)

        self.show()

        self.morrf_progress.start()
        self.connect(self.morrf_progress, QtCore.SIGNAL("MORRF_STATUS"), self.updateProgress)

    def updateProgress(self, data):
        if data >= 100:
            self.hide()
            self.morrf_progress.terminate()

        else:
            self.progress.setValue(data)
