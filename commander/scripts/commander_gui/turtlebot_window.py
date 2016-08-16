#!/usr/bin/env python

import sys, rospy, time

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from objective.premade_objectives import PremadeObjs

from error_popup.no_path_error import NoPath

from mm_apriltags_tracker.msg import april_tag_pos
from geometry_msgs.msg import Pose2D

APRILTAG = 100
TBOT = 32
HEIGHT = 550
WIDTH = 550
TURTLEBOT_OFFSET = 17 * 4

OBS_DICT = {30:(52,52), 31:(46, 54), 33:(54, 74), 34:(72, 100)}

class TurtlebotWindow(QtGui.QWidget):
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        rospy.init_node("turtlebot_window", anonymous = True)

        self.setWindowTitle("Turtlebot Window")

        self.resize(WIDTH, HEIGHT)

        self.tag_sub = rospy.Subscriber("/april_tag_pos", april_tag_pos, self.updateObjectPositions)

        self.black_pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)
        self.blue_pen = QtGui.QPen(QtCore.Qt.blue, 5, QtCore.Qt.SolidLine)
        self.fade_color = QtGui.QColor(QtCore.Qt.gray)
        self.fade_color.setAlpha(80)
        self.highlighter = QtCore.Qt.magenta

        self.pink_pen = QtGui.QPen(self.highlighter, 5, QtCore.Qt.SolidLine)
        self.fade_pen = QtGui.QPen(self.fade_color, 5, QtCore.Qt.SolidLine)

        self.agent_map = {}

        self.po = PremadeObjs()

        self.image_creation = False

        self.windowInit()

    def windowInit(self):

        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.white)
        self.setPalette(p)

        self.show()

    def getImage(self):
        pass

    def updateObjectPositions(self, msg):
        for i in range(len(msg.id)):
            self.agent_map[msg.id[i]] = msg.pose[i]

        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)

        for key, value in self.agent_map.iteritems():
            if key in OBS_DICT.keys():
                dim = OBS_DICT[key]

                width = dim[0] + TURTLEBOT_OFFSET
                height = dim[1] + TURTLEBOT_OFFSET

                rect = QRect( value.x - dim[0] / 2, value.y - dim[1] / 2, width, height)
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

        if hasattr(self, "morrf_paths") and self.image_creation == False:
            for i in range(len(self.morrf_paths.paths)):
                for index in range(len(self.morrf_paths.paths[i].waypoints)):

                    if i == self.path_index:
                        painter.setPen(self.pink_pen)

                    else:
                        painter.setPen(self.fade_pen)

                    path = self.morrf_paths.paths[i]

                    if index != 0:
                        p1 = QPoint(path.waypoints[index - 1].x, path.waypoints[index - 1].y)
                        p2 = QPoint(path.waypoints[index].x, path.waypoints[index].y)

                        painter.drawLine(p1, p2)

        painter.end()

    def getMORRFImage(self):
        self.image_creation = True

        self.update()
        QApplication.processEvents()

        p = QPixmap.grabWidget(self)
        img = p.toImage()

        self.image_creation = False
        self.update()

        return img

    def getGoalPoint(self):
        for key, value in self.agent_map.iteritems():
            if key == 51:
                return (value.x, value.y)

    def getStartPoint(self):
        for key, value in self.agent_map.iteritems():
            if key == 50:
                return (value.x, value.y)

    def getEnemyLocations(self):
        enemies = []

        for key, value in self.agent_map.iteritems():
            if key < 50 and key > 39:
                p = Pose2D()
                p.x = value.x
                p.y = value.y

                enemies.append(p)

        return enemies

    def keyPressEvent(self, event):
        key = event.key()
        if hasattr(self, "morrf_paths"):
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

    def startPathCycler(self, response):
        if hasattr(response, "paths") and len(response.paths) > 0:
            self.morrf_paths = response
            self.path_index = 0
            self.update()

        else:
            self.error = NoPath()

    def getMORRFPath(self):
        return self.morrf_paths.paths[self.path_index]


