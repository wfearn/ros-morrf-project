#!/usr/bin/env python

import os
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

GOAL_PATH = "/home/wfearn/catkin_ws/src/ros-morrf-project/commander/data/objective_icons/goal.png"
START_PATH = "/home/wfearn/catkin_ws/src/ros-morrf-project/commander/data/objective_icons/start.png"
ENEMY_PATH = "/home/wfearn/catkin_ws/src/ros-morrf-project/commander/data/objective_icons/enemy.png"

class Objective:
    def __init__(self, objective, xpos, ypos):
       
       print "Setting objective type to %s" % objective
       self.objective_type = objective

       print "objective type is %s " % self.objective_type

       self.setImage()

       self.x = xpos
       self.y = ypos


    def setImage(self):
        if self.objective_type == "start":
            self.image = QImage(START_PATH)

        elif self.objective_type == "goal":
            self.image = QImage(GOAL_PATH)
        else:
            self.image = QImage(ENEMY_PATH)

    def getImage(self):
        return self.image

    def getPosition(self):
        return (self.x, self.y)

    def getQPoint(self):
        point = QtGui.QPoint(self.x, self.y)
        return point

    def getObjectiveType(self):
        return self.objective_type

    def setPosition(self, xpos, ypos):
        self.x = xpos
        self.y = ypos

    def getDrawQPoint(self):
        point = QPoint(self.x - (self.image.width() / 2), self.y - (self.image.height() / 2))
        return point
