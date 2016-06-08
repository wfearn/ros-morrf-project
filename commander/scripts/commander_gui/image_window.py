#!/usr/bin/env python

import sys
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from enum import Enum
import os
from objective.objective import Objective
from morrf_ros.msg import *
from morrf_ros.srv import *
from error_popup.no_path_error import NoPath

IMAGE_FILE = "../../data/{}"
SET_START = "Select Start Position"
SET_GOAL = "Select Goal Position"
SET_ENEMIES = "Select Enemy Positions"

goal_file = "/home/wfearn/catkin_ws/src/ros-morrf-project/commander/data/objective_icons/goal.png"

class State(Enum):
    start = 0
    goal = 1
    enemies = 2

class Image(QtGui.QMainWindow):
    def __init__(self, image_name):
        super(Image, self).__init__()

        #keeps track of what point is
        #being set ie. start point, goal point, the enemy locations, etc.  
        self.state = State.start

        self.image_name = image_name

        self.setWindowTitle("Image")

        self.image = QPixmap(image_name)

        self.setGeometry(50, 50, self.image.width(), self.image.height())

        self.setContextMenuPolicy(Qt.ActionsContextMenu)

        goalAction = QAction("Set Goal...", self)
        startAction = QAction("Set Start...", self)
        enemyAction = QAction("Set Enemy...", self)
        resetAction = QAction("Reset", self)

        startAction.triggered.connect(self.createStart)
        goalAction.triggered.connect(self.createGoal)
        enemyAction.triggered.connect(self.createEnemy)
        resetAction.triggered.connect(self.reset)

        self.addAction(goalAction)
        self.addAction(startAction)
        self.addAction(enemyAction)
        self.addAction(resetAction)

        self.objectives = []

        palette = QPalette()
        palette.setBrush(QPalette.Background, QBrush(self.image))
        self.setPalette(palette)


        self.pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)

        self.show()

    def createStart(self):
        if not self.contains("start"):
            obj = Objective("start", self.mouseX, self.mouseY)
            self.objectives.append(obj)

        else:
            for obj in self.objectives:
                if obj.getObjectiveType() == "start":
                    obj.setPosition(self.mouseX, self.mouseY)

        self.update()

    def createGoal(self):
        if not self.contains("goal"):
            obj = Objective("goal", self.mouseX, self.mouseY)
            self.objectives.append(obj)

        else:
            for obj in self.objectives:
                if obj.getObjectiveType() == "goal":
                    obj.setPosition(self.mouseX, self.mouseY)

        self.update()

    def createEnemy(self):
        obj = Objective("enemy", self.mouseX, self.mouseY)
        self.objectives.append(obj)

        self.update()

    def mousePressEvent(self, QMouseEvent):
        mouse_state = QMouseEvent.button()

        self.mouseX = QMouseEvent.x()
        self.mouseY = QMouseEvent.y()

        for obj in self.objectives:
            if obj.isInsideBoundary((self.mouseX, self.mouseY)) and mouse_state == QtCore.Qt.LeftButton:
                obj.setClicked(True)

        self.update()

    def mouseMoveEvent(self, event):

        #Assumes mouse move event only occurs when LMB is held down
        for obj in self.objectives:
            if obj.isClicked():
                obj.setPosition(event.x(), event.y())

        self.update()

    def mouseReleaseEvent(self, event):
        for obj in self.objectives:
            if obj.isClicked():
                obj.setClicked(False)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)


        painter.setPen(self.pen)

        for obj in self.objectives:
            img = obj.getImage()
            painter.drawImage(obj.getDrawQPoint(), img)

        if hasattr(self, 'morrf_paths'):
            for path in self.morrf_paths.paths:
                #print "There are %s paths" % len(self.morrf_paths.paths)
                for index in range(len(path.waypoints)):
                    #print "In this path there are %s waypoints" % len(path.waypoints)
                    if index != 0:
                        point1 = QPoint(path.waypoints[index - 1].x, path.waypoints[index - 1].y)
                        point2 = QPoint(path.waypoints[index].x, path.waypoints[index].y)

                        painter.drawLine(point1, point2)

        painter.end()

    def reset(self):
        self.objectives = []
        self.update()

    def mouseInitialized(self):
        return hasattr(self, 'mouseX') and hasattr(self, 'mouseY')

    def isCompleted(self):
        return (self.contains("start") and self.contains("goal") and self.contains("enemy"))

    def getStartPoint(self):
        for obj in self.objectives:
            if obj.getObjectiveType() == "start":
                return obj.getPosition()

    def getGoalPoint(self):
        for obj in self.objectives:
            if obj.getObjectiveType() == "goal":
                return obj.getPosition()

    def getEnemyLocations(self):
        enemies = []
        for obj in self.objectives:
            if obj.getObjectiveType() == "enemy":
                enemies.append(obj.getPosition())

        return enemies

    def saveMapToDropbox(self):
        pass

    def getMapName(self):
        return self.image_name

    def contains(self, objective):
        for obj in self.objectives:
            if obj.getObjectiveType() == objective:
                return True

        return False

    def printMorrfPaths(self, response):
        if len(response.paths) > 0:
            self.morrf_paths = response
            self.update()
        else:
            self.error = NoPath()

    def delMorrfPaths(self):
        if hasattr(self, 'morrf_paths'):
            delattr(self, 'morrf_paths')
