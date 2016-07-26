#!/usr/bin/env python

import os
from PyQt4 import QtGui
from PyQt4 import QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *
import rospkg

GOAL_PATH = "{}/data/objective_icons/goal.png"
START_PATH = "{}/data/objective_icons/start.png"
ENEMY_PATH = "{}/data/objective_icons/enemy.png"
ROBOT_PATH = "{}/data/objective_icons/robot1.png"

#TODO:Create an Objectives class that manages objectives

class PremadeObjs:
    def __init__(self):

        rospack = rospkg.RosPack()
        self.path = rospack.get_path('commander')

        self.enemy_image = QImage(ENEMY_PATH.format(self.path))
        self.start_image = QImage(START_PATH.format(self.path))
        self.goal_image = QImage(GOAL_PATH.format(self.path))

    def getEnemyDrawPoint(self, x, y):
        return self.getDrawQPoint(x, y, self.enemy_image)

    def getStartDrawPoint(self, x, y):
        return self.getDrawQPoint(x, y, self.start_image)

    def getGoalDrawPoint(self, x, y):
        return self.getDrawQPoint(x, y, self.goal_image)

    def getImage(self):
        return self.image

    def getDrawQPoint(self, x, y, image):
        point = QPoint( (x - (image.width() / 2)), (y - (image.height() / 2)) )
        return point

    def getEnemyImage(self):
        return self.enemy_image

    def getStartImage(self):
        return self.start_image

    def getGoalImage(self):
        return self.goal_image
