#!/usr/bin/env python

import os, sys, rospy
import math as math

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from objective.objective import Objective
from error_popup.impediment_error import ImpedimentError

from objective.premade_objectives import PremadeObjs

from error_popup.no_path_error import NoPath

from mm_apriltags_tracker.msg import april_tag_pos
from geometry_msgs.msg import Pose2D

EPSILON = 1

APRILTAG = 100
TBOT = 32
HEIGHT = 550
WIDTH = 550

OBS_DICT = {30:(52, 52), 31:(46, 54), 33:(54, 74), 34:(72, 100), 35:(50, 50)}

MORRF_OUTPUT_FILE = "/home/wfearn/Dropbox/MORRF_OUTPUT/morrf_output/{}"
IMG_OUTPUT_FILE = "/home/wfearn/Dropbox/MORRF_OUTPUT/maps/{}"

PATHS_FILE = MORRF_OUTPUT_FILE.format("paths.txt")
COSTS_FILE = MORRF_OUTPUT_FILE.format("costs.txt")


class Sketch(QtGui.QMainWindow):
    def __init__(self):
        super(Sketch, self).__init__()

        self.setWindowTitle("Sketch GUI")

        self.setGeometry(50, 50, WIDTH, HEIGHT)

        self.tag_sub = rospy.Subscriber("/april_tag_pos", april_tag_pos, self.updateObjectPositions)

        self.agent_map = {}
        self.waypoints = []

        self.po = PremadeObjs()

        self.image_creation = False

        self.setContextMenuPolicy(Qt.ActionsContextMenu)

        resetAction = QAction("Reset", self)
        resetAction.triggered.connect(self.reset)

        self.addAction(resetAction)

        self.fade_color = QtGui.QColor(QtCore.Qt.gray)
        self.fade_color.setAlpha(80)
        self.highlighter = QtCore.Qt.magenta

        self.black_pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)
        self.red_pen = QtGui.QPen(QtCore.Qt.red, 5, QtCore.Qt.SolidLine)
        self.fade_pen = QtGui.QPen(self.fade_color, 5, QtCore.Qt.SolidLine)
        self.pink_pen = QtGui.QPen(self.highlighter, 5, QtCore.Qt.SolidLine)

        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.white)
        self.setPalette(p)

        self.show()

    def reset(self):
        self.waypoints = []

        self.update()

    def mousePressEvent(self, event):

        mouse_state = event.button()

        self.x = event.x()
        self.y = event.y()

        if mouse_state == QtCore.Qt.LeftButton:
            if self.freeOfObstacle(self.x, self.y):
                self.waypoints.append((self.x, self.y))
            else:
                self.error = ImpedimentError()

        self.update()

    def getImage(self):
        self.image_creation = True

        self.update()
        QApplication.processEvents()

        p = QPixmap.grabWidget(self)
        self.img = p.toImage()

        self.image_creation = False
        self.update()

        return self.img

    def freeOfObstacle(self, x, y):

        img = self.getImage()

        if len(self.waypoints) > 1:

            color = QColor(img.pixel(x, y))

            if color == QtCore.Qt.black:
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

                    if color == QtCore.Qt.black:
                        return False

                return True
        else:

            if img.pixel(x, y) == QtCore.Qt.black:
                return False

            else:
                return True

    def distance(self, p1, p2):
        return math.sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 )

    def updateObjectPositions(self, msg):
        for i in range(len(msg.id)):
            self.agent_map[msg.id[i]] = msg.pose[i]

        self.update()

    def hasStart(self):
        for key, value in self.agent_map.iteritems():
            if key == 50:
                return True

        return False

    def hasGoal(self):
        for key, value in self.agent_map.iteritems():
            if key == 51:
                return True

        return False

    def paintEvent(self, event):

        painter = QtGui.QPainter()
        painter.begin(self)
        painter.setPen(self.black_pen)

        for key, value in self.agent_map.iteritems():
            if key in OBS_DICT.keys():
                dim = OBS_DICT[key]

                rect = QRect( (value.x - dim[0] / 2), (value.y - dim[1] / 2), dim[0], dim[1])
                painter.setBrush(Qt.black)
                painter.drawRect(rect)

            elif key < 50 and key > 39 and self.image_creation == False:
                painter.drawImage( self.po.getEnemyDrawPoint(value.x, value.y), self.po.getEnemyImage() )

            elif key == 50 and self.image_creation == False:
                painter.drawImage( self.po.getStartDrawPoint(value.x, value.y), self.po.getStartImage() )

            elif key == 51 and self.image_creation == False:
                painter.drawImage( self.po.getGoalDrawPoint(value.x, value.y), self.po.getGoalImage() )

            elif key == 97 and self.image_creation == False:
                painter.drawImage( self.po.getRoboDrawPoint(value.x, value.y), self.po.getRoboImage() )

        painter.setPen(self.red_pen)

        for i in range(len(self.waypoints)):
            if i != 0:
                p1 = QPoint(self.waypoints[i - 1][0], self.waypoints[i - 1][1])
                p2 = QPoint(self.waypoints[i][0], self.waypoints[i][1])

                painter.drawLine(p1, p2)

        if hasattr(self, "tarrt_paths") and self.image_creation == False:
            for i in range(len(self.tarrt_paths.paths)):
                for index in range(len(self.tarrt_paths.paths[i].waypoints)):

                    if i == self.path_index:
                        painter.setPen(self.pink_pen)

                    else:
                        painter.setPen(self.fade_pen)

                    path = self.tarrt_paths.paths[i]

                    if index != 0:
                        p1 = QPoint(path.waypoints[index - 1].x, path.waypoints[index - 1].y)
                        p2 = QPoint(path.waypoints[index].x, path.waypoints[index].y)

                        painter.drawLine(p1, p2)

        painter.end()

    def getEnemyLocations(self):
        enemies = []

        for key, value in self.agent_map.iteritems():
            if key < 50 and key > 39:
                p = Pose2D()
                p.x = value.x
                p.y = value.y

                enemies.append(p)

        return enemies


    def getGoalPoint(self):
        for key, value in self.agent_map.iteritems():
            if key == 51:
                return (value.x, value.y)

    def getStartPoint(self):
        for key, value in self.agent_map.iteritems():
            if key == 50:
                return (value.x, value.y)

    def startPathCycler(self, paths):
        if hasattr(paths, "paths") and len(paths.paths) > 0:
            self.tarrt_paths = paths
            self.path_index = 0
            self.update()

        else:
            self.error = NoPath()
