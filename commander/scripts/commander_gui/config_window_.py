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

class Window(QtGui.QMainWindow):

  def __init__(self):
    super(Window, self).__init__()

    #QtGui.QApplication.setStyle(QtGui.QStyleFactory.create("Plastique"))

        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Simple Config")
        self.center()

        action = QtGui.QAction("Close", self)
        action.setShortcut("Ctrl+C")
        action.triggered.connect (QtCore.QCoreApplication.instance().quit)

        self.addAction(action)

        self.statusBar()

        self.green_pen = QtGui.QPen(QtCore.Qt.green, 5, QtCore.Qt.SolidLine)

        self.objectives = []

        self.view()

  def view(self):
    self.imgLoad = QtGui.QPushButton("Load Image", self)
    self.imgLoad.clicked.connect(self.load_image)

    btn = QtGui.QPushButton("Exit", self)
    btn.clicked.connect(QtCore.QCoreApplication.instance().quit)

    self.robot = QtGui.QPushButton("Send Robot", self)
    self.robot.setEnabled(False)
    self.robot.clicked.connect(self.send_robot)

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
    layout.addWidget(self.imgLoad)
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

  def send_robot(self):

      index = self.image_window.getPathIndex()
      path = self.morrf_response.paths[index].waypoints


      if self.image_window.getRobot() == None:
          self.image_window.createRobot((self.start.x, self.start.y))
      else:
          self.image_window.updateRobPosition( (self.start.x, self.start.y) )

      rob = self.image_window.getRobot()

      v = .1


      for point in path:

          p1 = (point.x, point.y)
          p2 = rob.getPosition()


          while self.dist(p1, p2) > 1:
              theta = math.atan2( (p1[1] - p2[1]), (p1[0] - p2[0]) )
              x = v * math.cos(theta)
              y = v * math.sin(theta)

              p2 = (p2[0] + x, p2[1] + y)
              self.image_window.updateRobPosition( p2 )
              QtCore.QCoreApplication.instance().processEvents()


  def dist(self, p1, p2):
      return math.sqrt( (p2[0] - p1[0]) **2 + (p2[1] - p1[1]) **2)


  def load_image(self):
      self.image_name = QtGui.QFileDialog.getOpenFileName(self, "Load Image")
      self.image_window = Image(self.image_name)

      self.robot.setEnabled(False)

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
      if not hasattr(self, "image_window"):
          self.error1 = NoImage()

          resolution = QtGui.QDesktopWidget().screenGeometry()

          self.error1.move((resolution.width() / 2), (resolution.height() / 2))

      elif not self.image_window.isCompleted():
          self.error2 = NotInitialized()
      else:
          self.image_window.delMorrfPaths()
          self.initialize()

  def initialize(self):
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

          initializer.number_of_iterations = 1000
          initializer.segment_length = 5
          initializer.number_of_trees = 5
          initializer.objective_number = self.quick.isChecked() + self.safe.isChecked() + self.stealth.isChecked()
          initializer.minimum_distance_enabled = self.quick.isChecked()
          initializer.method_type = 0 #Weighted Sum

          self.costmap_response = StartCostmapPublisher(initializer.map, self.stealth.isChecked(), self.safe.isChecked(), self.image_window.getEnemyLocations())

          initializer.cost_maps = self.costmap_response.cost_maps

          self.morrf_response = StartCommanderPublisher(initializer)

          self.image_window.startPathCycler(self.morrf_response)

          if hasattr(self.image_window, 'morrf_paths'):
              self.robot.setEnabled(True)
          else:
              self.robot.setEnabled(False)

  def map_convert(self):
      if hasattr(self, "image_name"):

          img = cv2.imread(str(self.image_name))

          image = int16_image()
          image.width = img.shape[1]
          image.height = img.shape[0]
          image.name = str(self.image_name)

          for j in range (image.height):
              for i in range (image.width):
                  image.int_array.append(np.int16(img[j, i, 0]))
          return image

      else:
          return None

def main():
  app = QtGui.QApplication(sys.argv)
  GUI = Window()
  #GUI.show()
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
