#!/usr/bin/env python

import sys
import rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from image_window import Image

from morrf_ros.msg import *

import cv2
import numpy as np
import math

from camera_window import CameraWindow

from objective.objective import Objective

from error_popup.not_initialized import NotInitialized
from error_popup.no_image_selected import NoImage

from publishers.costmap_publisher import StartCostmapPublisher
from publishers.commander_publisher import StartCommanderPublisher

STARTX = 1200
STARTY = 1200
WIDTH = 200
HEIGHT = 300
ROBOT_PATH = "{}/data/objective_icons/robot.png"

class CameraConfig(QtGui.QMainWindow):

  def __init__(self):
    super(CameraConfig, self).__init__()

    #QtGui.QApplication.setStyle(QtGui.QStyleFactory.create("Plastique"))

    self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
    self.setWindowTitle("Camera Config")
    self.center()

    self.image_window = CameraWindow()

    action = QtGui.QAction("Close", self)
    action.setShortcut("Ctrl+C")
    action.triggered.connect(QtCore.QCoreApplication.instance().quit)

    self.addAction(action)

    self.statusBar()

    self.view()

  def view(self):
    btn = QtGui.QPushButton("Exit", self)
    btn.clicked.connect(QtCore.QCoreApplication.instance().quit)

    self.robot = QtGui.QPushButton("Send Robot", self)
    self.robot.setEnabled(False)

    self.quick = QtGui.QCheckBox("Quickly", self)
    self.quick.setChecked(False)
    #self.quick.setStatusTip('Need a Quick Path')
    self.quick.stateChanged.connect(self.enable_launch)

    self.stealth = QtGui.QCheckBox("Stealthily", self)
    self.stealth.setChecked(False)
    self.stealth.stateChanged.connect(self.enable_launch)

    self.safe = QtGui.QCheckBox("Safely", self)
    self.safe.setChecked(False)
    self.safe.stateChanged.connect(self.enable_launch)

    self.launch = QtGui.QPushButton("Launch MORRF", self)
    self.launch.setEnabled(False)
    self.launch.clicked.connect(self.launch_morrf)

    layout = QVBoxLayout()
    layout.addWidget(self.quick)
    layout.addWidget(self.stealth)
    layout.addWidget(self.safe)
    layout.addWidget(self.launch)
    layout.addWidget(self.robot)
    layout.addWidget(btn)

    self.widget = QtGui.QWidget(self)
    self.widget.setLayout(layout)
    self.setCentralWidget(self.widget)

    self.show()

  def enable_launch(self, state):
      if state == QtCore.Qt.Checked:
          self.launch.setEnabled(True)

      elif self.quick.checkState() or self.stealth.checkState() or self.safe.checkState():
          self.launch.setEnabled(True)

      else:
          self.launch.setEnabled(False)


  def center(self):
      frameGm = self.frameGeometry()
      centerPoint = QtGui.QDesktopWidget().availableGeometry().center()
      frameGm.moveCenter(centerPoint)
      self.move(frameGm.topLeft())

  def launch_morrf(self):
          self.initialize()

  def initialize(self):
          print "Launching MORRF..."

          goal = self.image_window.getGoalPoint()
          start = self.image_window.getStartPoint()

          initializer = morrf_init()

          initializer.goal.x = goal[0]
          initializer.goal.y = goal[1]
          initializer.start.x = start[0]
          initializer.start.y = start[1]

          self.start = initializer.start

          initializer.map = self.map_convert()
          initializer.width = initializer.map.width
          initializer.height = initializer.map.height

          initializer.number_of_iterations = 2000
          initializer.segment_length = 5
          initializer.number_of_trees = 5
          initializer.objective_number = self.quick.isChecked() + self.safe.isChecked() + self.stealth.isChecked()
          initializer.minimum_distance_enabled = self.quick.isChecked()
          initializer.method_type = 0 #Weighted Sum

          print "Sending map to costmap generator..."
          self.costmap_response = StartCostmapPublisher(initializer.map, self.stealth.isChecked(), self.safe.isChecked(), self.image_window.getEnemyLocations())

          initializer.cost_maps = self.costmap_response.cost_maps

          print "Sending data to MORRF..."
          self.morrf_response = StartCommanderPublisher(initializer)

          #self.image_window.startPathCycler(self.morrf_response)

          if hasattr(self.image_window, 'morrf_paths'):
              self.robot.setEnabled(True)
          else:
              self.robot.setEnabled(False)

  def map_convert(self):
      img = self.image_window.getMORRFImage()

      image = int16_image()
      image.width = img.width()
      image.height = img.height()

      for i in range(img.height()):
          for j in range(img.width()):
              c = QColor(img.pixel(j, i))

              image.int_array.append(c.blue())

      return image

