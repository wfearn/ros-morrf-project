#!/usr/bin/env python

import sys

from publishers.tarrt_publisher import StartTarrtPublisher

from tarrt_ros.msg import *
from tarrt_ros.srv import *

from PyQt4 import QtGui, QtCore


class TarrtThread(QtCore.QThread):
    def __init__(self, tarrt_init, parent = None):
        super(TarrtThread, self).__init__(parent)

        self.init = tarrt_init


    def run(self):
        response = StartTarrtPublisher(self.init)

        self.emit(QtCore.SIGNAL("TARRT_RESPONSE"), response)
