#!/usr/bin/env python

import sys

from publishers.continue_tarrt_publisher import StartContinueTarrtPublisher

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from tarrt_ros.msg import *
from tarrt_ros.srv import *

class ContinueTarrtThread(QtCore.QThread):
    def __init__(self, iterations, parent = None):
        super(ContinueTarrtThread, self).__init__(parent)

        self.iters = iterations

    def run(self):
        response = StartContinueTarrtPublisher(self.iters)

        self.emit(QtCore.SIGNAL("CONTINUE_TARRT_RESPONSE"), response)
