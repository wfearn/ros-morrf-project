#!/usr/bin/env python

import sys

from publishers.continue_publisher import StartContinuePublisher

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from morrf_ros.msg import *
from morrf_ros.srv import *

class ContinueThread(QtCore.QThread):
    def __init__(self, iterations, parent = None):
        super(ContinueThread, self).__init__(parent)

        self.iters = iterations

    def run(self):
        response = StartContinuePublisher(self.iters)

        self.emit(QtCore.SIGNAL("CONTINUE_RESPONSE"), response)
