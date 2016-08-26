#!/usr/bin/env python

import sys

from publishers.harrt_publisher import StartHarrtPublisher

from harrt_ros.msg import *
from harrt_ros.srv import *

from PyQt4 import QtGui, QtCore


class HarrtThread(QtCore.QThread):
    def __init__(self, harrt_init, parent = None):
        super(HarrtThread, self).__init__(parent)

        self.init = harrt_init


    def run(self):
        response = StartHarrtPublisher(self.init)

        self.emit(QtCore.SIGNAL("HARRT_RESPONSE"), response)
