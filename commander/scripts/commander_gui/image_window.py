#!/usr/bin/env python

import sys
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from enum import Enum

IMAGE_FILE = "../../data/{}"
SET_START = "Select Start Position"
SET_GOAL = "Select Goal Position"
SET_ENEMIES = "Select Enemy Positions"

class State(Enum):
    start = 0 
    goal = 1 
    enemies = 2 

class Image(QtGui.QMainWindow):
    def __init__(self, image_name):
        super(Image, self).__init__()

        #Setting up an enum to keep track of what point is 
        #being set ie. start point, goal point, the enemy locations, etc.

        self.state = State.start

        self.setWindowTitle("Image")

        self.image = QPixmap(image_name)


        self.setGeometry(50, 50, self.image.width(), self.image.height())

        palette = QPalette()
        palette.setBrush(QPalette.Background, QBrush(self.image))
        self.setPalette(palette)

        self.statusBar().showMessage(SET_START)

        reset_btn = QtGui.QPushButton("Reset", self)
        reset_btn.resize(100, 25)
        reset_btn.move(self.image.width() - 100, self.image.height() - 25)
        reset_btn.clicked.connect(self.reset)

        set_btn = QtGui.QPushButton("Set", self)
        set_btn.resize(75, 25)
        set_btn.move(175, self.image.height() - 25)
        set_btn.clicked.connect(self.setPoint)

        self.pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)
        self.draw_points = []

        self.show()

    def mousePressEvent(self, QMouseEvent):
        self.mouseX = QMouseEvent.x()
        self.mouseY = QMouseEvent.y()

        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)


        painter.setPen(self.pen)

        #For when paint events happens before user clicks on the image
        if self.mouseInitialized():
            painter.drawPoint(self.mouseX, self.mouseY)

        for index in range(len(self.draw_points)):
            point = self.draw_points[index]

            #If we're drawing enemies, we want red dots instead of black ones
            if index > 1:
                redpen = QtGui.QPen(QtCore.Qt.red, 5, QtCore.Qt.SolidLine)
                painter.setPen(redpen)
                painter.drawPoint(point[0], point[1])
            else:
                painter.drawPoint(point[0], point[1])

        painter.end()

    def reset(self):
        self.statusBar().showMessage(SET_START)

        #self.pen = QtGui.QPen(QtCore.Qt.blue, 5, QtCore.Qt.SolidLine)

        self.draw_points = []
        self.update()

    def setPoint(self):
        if self.state == State.start:
            if self.mouseInitialized():

                self.statusBar().showMessage(SET_GOAL)
                self.draw_points.append((self.mouseX, self.mouseY))

                self.state = State.goal

        elif self.state == State.goal:

            self.statusBar().showMessage(SET_ENEMIES)

            self.draw_points.append((self.mouseX, self.mouseY))


            self.state = State.enemies
        else:

            self.draw_points.append((self.mouseX, self.mouseY))

        self.update()

    def mouseInitialized(self):
        return hasattr(self, 'mouseX') and hasattr(self, 'mouseY')

    def isCompleted(self):
        return len(self.draw_points) > 2

    def getStartPoint(self):
        if self.draw_points[0] != None:
            return self.draw_points[0]
        else:
            return None

    def getGoalPoint(self):
        if self.draw_points[1] != None:
            return self.draw_points[1]
        else:
            return None

    def getEnemyLocations(self):
        if len(self.draw_points) > 2:
            enemies = []
            for i in range(len(self.draw_points)):
                if i > 1:
                    enemies.append(self.draw_points[i])

            return enemies
        else:
            return None

