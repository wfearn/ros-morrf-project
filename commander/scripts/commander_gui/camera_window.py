#!/usr/bin/env python

import sys, rospy, time

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from objective.premade_objectives import PremadeObjs

from mm_apriltags_tracker.msg import april_tag_pos
from geometry_msgs.msg import Pose2D

APRILTAG = 6
TBOT = 32
HEIGHT = 600
WIDTH = 600

class CameraWindow(QtGui.QWidget):
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        rospy.init_node("camera_window", anonymous = True)

        self.setWindowTitle("Camera Window")

        self.resize(WIDTH, HEIGHT)

        self.tag_sub = rospy.Subscriber("/april_tag_pos", april_tag_pos, self.updateObjectPositions)

        self.black_pen = QtGui.QPen(QtCore.Qt.black, 5, QtCore.Qt.SolidLine)
        self.blue_pen = QtGui.QPen(QtCore.Qt.blue, 5, QtCore.Qt.SolidLine)

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
            if key < 40 and key > 29:
                rect = QRect( (value.x - APRILTAG / 2), (value.y - APRILTAG / 2), APRILTAG, APRILTAG )
                painter.setBrush(Qt.black)
                painter.drawRect(rect)

            elif key < 50 and key > 39 and self.image_creation == False:
                painter.drawImage( self.po.getEnemyDrawPoint(value.x, value.y), self.po.getEnemyImage() )

            elif key == 50 and self.image_creation == False:
                painter.drawImage( self.po.getStartDrawPoint(value.x, value.y), self.po.getStartImage() )

            elif key == 51 and self.image_creation == False:
                painter.drawImage( self.po.getGoalDrawPoint(value.x, value.y), self.po.getGoalImage() )

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
