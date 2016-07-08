#!/usr/bin/env python

from enum import Enum
import os
import sys

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from objective.objective import Objective
from error_popup.no_path_error import NoPath

from morrf_ros.msg import *
from morrf_ros.srv import *
from geometry_msgs.msg import *

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

        self.black_pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)
        self.green_pen = QtGui.QPen(QtCore.Qt.green, 5, QtCore.Qt.SolidLine)

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

    def createRobot(self, start):
        #if not self.contains("robot"):
        #print ("does not contain robot")
        obj = Objective("robot", start[0], start[1])
        self.objectives.append(obj)
        self.update()

    def getRobot(self):
	for obj in self.objectives:
		if obj.getObjectiveType() == 'robot':
			return obj
	return None

    def updateRobPosition(self, p):
	for obj in self.objectives:
		if obj.getObjectiveType() == 'robot':
			obj.setPosition(p[0], p[1])

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

    def keyPressEvent(self, event):
        key = event.key()
        if hasattr(self, 'morrf_paths'):
            if key == QtCore.Qt.Key_Left:
                if self.path_index == 0:
                    self.path_index = ( len(self.morrf_paths.paths) - 1 )

                else:
                    self.path_index -= 1

            if key == QtCore.Qt.Key_Right:
                if self.path_index == ( len(self.morrf_paths.paths) - 1 ):
                    self.path_index = 0

                else:
                    self.path_index += 1

        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)


        painter.setPen(self.green_pen)

        for obj in self.objectives:
            img = obj.getImage() 
            painter.drawImage(obj.getDrawQPoint(), img)
	    
	if hasattr(self, 'morrf_paths'):
            for index in range(len(self.morrf_paths.paths[self.path_index].waypoints)):

                path = self.morrf_paths.paths[self.path_index]

                #Inefficient to draw line between first point and itself
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

                p = Pose2D()
                p.x = obj.getPosition()[0]
                p.y = obj.getPosition()[1]

                enemies.append(p)

        return enemies

    def saveMapToDropbox(self):
        i = QImage(self.image_name)

        i.save("/home/wfearn/Dropbox/MORRF_OUTPUT/maps/map.png")

    def getMapName(self):
        return str(self.image_name)

    def contains(self, objective):
        for obj in self.objectives:
            if obj.getObjectiveType() == objective:
                return True

        return False

    def startPathCycler(self, response):
        if hasattr(response, "paths") and len(response.paths) > 0:
            self.showStatusBar()
            self.morrf_paths = response
            self.path_index = 0
            self.update()

        else:
            self.error = NoPath()

    def delMorrfPaths(self):
        if hasattr(self, 'morrf_paths'):
            delattr(self, 'morrf_paths')
            self.update()

    def showStatusBar(self):
        self.statusBar().showMessage("Use the arrow keys to cycle between path options")
