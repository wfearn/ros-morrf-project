#!/usr/bin/env python

import sys, rospy

from PyQt4 import QtGui, QtCore

from std_msgs.msg import Float64

class ProgressBarThread(QtCore.QThread):
    def __init__(self, parent = None):
        super(ProgressBarThread, self).__init__(parent)

    def run(self):
        self.progress = rospy.Subscriber("/morrf_status", Float64, self.emitProgress)

        rospy.spin()

    def emitProgress(self, data):

        update = int(data.data)

        self.emit(QtCore.SIGNAL("MORRF_STATUS"), update)
