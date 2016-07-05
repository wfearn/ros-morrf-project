#!/usr/bin/env python

import os
import sys
import math as math

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from objective.objective import Objective
from error_popup.impediment_error import ImpedimentError

EPSILON = 1

class Sketch(QtGui.QMainWindow):
    def __init__(self, image_name):
        super(Sketch, self).__init__()

        self.image_name = image_name

        self.setWindowTitle("Sketch GUI")

        self.image = QPixmap(image_name)

        self.setGeometry(50, 50, self.image.width(), self.image.height())

        palette = QPalette()
        palette.setBrush(QPalette.Background, QBrush(self.image))
        self.setPalette(palette)

        self.setContextMenuPolicy(Qt.ActionsContextMenu)

        goalAction = QAction("Set Goal...", self)
        startAction = QAction("Set Start...", self)
        enemyAction = QAction("Set Enemy...", self)
        resetAction = QAction("Reset", self)
        toggleAction = QAction("Toggle Draw State", self)

        startAction.triggered.connect(self.createStart)
        goalAction.triggered.connect(self.createGoal)
        enemyAction.triggered.connect(self.createEnemy)
        resetAction.triggered.connect(self.reset)
        toggleAction.triggered.connect(self.toggleDrawState)

        self.addAction(goalAction)
        self.addAction(startAction)
        self.addAction(enemyAction)
        self.addAction(resetAction)
        self.addAction(toggleAction)

        self.objectives = []
        self.waypoints = []

        self.statusBar().showMessage("Select Objectives")
        self.draw_state = False

        self.black_pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)
        self.red_pen = QtGui.QPen(QtCore.Qt.red, 5, QtCore.Qt.SolidLine)

        self.show()

    def createStart(self):
        if not self.contains("start"):
            obj = Objective("start", self.x, self.y)
            self.objectives.append(obj)
        else:
            for obj in self.objectives:
                if obj.getObjectiveType() == "start":
                    obj.setPosition(self.x, self.y)

        self.update()

    def createGoal(self):
        if not self.contains("goal"):
            obj = Objective("goal", self.x, self.y)
            self.objectives.append(obj)
        else:
            for obj in self.objectives:
                if obj.getObjectiveType() == "goal":
                    obj.setPosition(self.x, self.y)

        self.update()

    def createEnemy(self):
        obj = Objective("enemy", self.x, self.y)
        self.objectives.append(obj)

        self.update()

    def reset(self):
        if not self.draw_state:
            self.objectives = []

        else:
            self.waypoints = []

        self.update()

    def mousePressEvent(self, event):

        mouse_state = event.button()

        self.x = event.x()
        self.y = event.y()

        if not self.draw_state:
            for obj in self.objectives:
                if obj.isInsideBoundary((self.x, self.y)) and mouse_state == QtCore.Qt.LeftButton:
                    obj.setClicked(True)

        elif mouse_state == QtCore.Qt.LeftButton:
            if self.freeOfObstacle(self.x, self.y):
                self.waypoints.append((self.x, self.y))
            else:
                self.error = ImpedimentError()

        self.update()

    def freeOfObstacle(self, x, y):

        img = self.image.toImage()

        if len(self.waypoints) > 1:

            color = QColor(img.pixel(x, y))

            if color.blue() == 0:
                return False

            else:
                index = len(self.waypoints) - 1

                p = self.waypoints[index]

                while self.distance(p, (x, y)) > 1:

                    theta = math.atan2(y - p[1], x - p[0])

                    nx = p[0] + EPSILON * math.cos(theta)
                    ny = p[1] + EPSILON * math.sin(theta)

                    p = (nx, ny)

                    color = QColor(img.pixel(p[0], p[1]))

                    if color.blue() == 0:
                        return False

                return True
        else:

            if img.pixel(x, y) == 0:
                return False

            else:
                return True

    def distance(self, p1, p2):
        return math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 )

    def mouseMoveEvent(self, event):

        if not self.draw_state:
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
        painter.setPen(self.red_pen)

        for obj in self.objectives:
            img = obj.getImage()
            painter.drawImage(obj.getDrawQPoint(), img)

        for i in range(len(self.waypoints)):
            if i != 0:
                p1 = QPoint(self.waypoints[i - 1][0], self.waypoints[i - 1][1])
                p2 = QPoint(self.waypoints[i][0], self.waypoints[i][1])

                painter.drawLine(p1, p2)

        painter.end()

    def contains(self, objective):
        for obj in self.objectives:
            if obj.getObjectiveType() == objective:
                return True

        return False

    def setStatusBarMessage(self, message):
        self.statusBar().showMessage(message)

    def toggleDrawState(self):
        if self.draw_state == False:
            self.draw_state = True
            self.statusBar().showMessage("Select Waypoint Coordinates")

        else:
            self.draw_state = False
            self.statusBar().showMessage("Select Objectives")

