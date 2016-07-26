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

class Objective:
    def __init__(self, objective, xpos, ypos):

       self.objective_type = objective

       self.setImage()

       self.x = xpos
       self.y = ypos

       self.clicked = False

    def setImage(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('commander')

        if self.objective_type == "start":
            self.image = QImage(START_PATH.format(path))

        elif self.objective_type == "goal":
            self.image = QImage(GOAL_PATH.format(path))

        elif self.objective_type == "robot":
            self.image = QImage(ROBOT_PATH.format(path))

        else:
            self.image = QImage(ENEMY_PATH.format(path))

    def setClicked(self, boolean):
        self.clicked = boolean

    def isClicked(self):
        return self.clicked

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
        #sets image center to draw point
        point = QPoint( (self.x - (self.image.width() / 2)), (self.y - (self.image.height() / 2)) )
        return point

    def isInsideBoundary(self, mouse_coordinate):
        half_width = self.image.width() / 2
        half_height = self.image.height() / 2

        left_side_x = self.x - half_width
        right_side_x = self.x + half_width
        top_side_y = self.y - half_height
        bottom_side_y = self.y + half_height

        if mouse_coordinate[0] > left_side_x and mouse_coordinate[0] < right_side_x:
            if mouse_coordinate[1] > top_side_y and mouse_coordinate[1] < bottom_side_y:
                return True
            else:
                return False
        else:
            return False
