#!/usr/bin/env python

import sys

from publishers.costmap_publisher import StartCostmapPublisher

from morrf_ros.msg import *
from morrf_ros.srv import *

from commander.msg import *
from commander.srv import *

from PyQt4 import QtGui, QtCore


class CostmapThread(QtCore.QThread):
    def __init__(self, map_img, stealth, safe, locs, parent = None):
        super(CostmapThread, self).__init__(parent)

        self.img = map_img
        self.stealth = stealth
        self.safe = safe
        self.locs = locs


    def run(self):
        response = StartCostmapPublisher(self.img, self.stealth, self.safe, self.locs)

        self.emit(QtCore.SIGNAL("COSTMAP_RESPONSE"), response)
