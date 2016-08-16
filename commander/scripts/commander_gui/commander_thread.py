#!/usr/bin/env python

import sys

from publishers.commander_publisher import StartCommanderPublisher

from morrf_ros.msg import *
from morrf_ros.srv import *

from PyQt4 import QtGui, QtCore


class CommanderThread(QtCore.QThread):
    def __init__(self, morrf_init, parent = None):
        super(CommanderThread, self).__init__(parent)

        self.init = morrf_init


    def run(self):
        response = StartCommanderPublisher(self.init)

        self.emit(QtCore.SIGNAL("MORRF_RESPONSE"), response)
