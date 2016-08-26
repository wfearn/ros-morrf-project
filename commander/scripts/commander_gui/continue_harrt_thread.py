#!/usr/bin/env python

import sys

from publishers.continue_harrt_publisher import StartContinueHarrtPublisher

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from harrt_ros.msg import *
from harrt_ros.srv import *

class ContinueHarrtThread(QtCore.QThread):
    def __init__(self, iterations, parent = None):
        super(ContinueHarrtThread, self).__init__(parent)

        self.iters = iterations

    def run(self):
        response = StartContinueHarrtPublisher(self.iters)

        self.emit(QtCore.SIGNAL("CONTINUE_HARRT_RESPONSE"), response)
