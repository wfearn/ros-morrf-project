#!/usr/bin/env python

import sys
import rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from sketch_window import Sketch

STARTX = 1000
STARTY = 1000
WIDTH = 250
HEIGHT = 200

class SketchConfig(QtGui.QMainWindow):

    def __init__(self):
        super(SketchConfig, self).__init__()
        self.setGeometry(STARTX, STARTY, WIDTH, HEIGHT)
        self.setWindowTitle("Sketch Config")

        self.img_load = QtGui.QAction("Load Image", self)
        self.img_load.triggered.connect(self.load_image)
        self.img_load.setShortcut("Ctrl+O")

        self.app_quit = QtGui.QAction("Quit", self)
        self.app_quit.triggered.connect(qApp.quit)
        self.app_quit.setShortcut("Ctrl+Q")

        main_menu = self.menuBar()

        self.quick = QtGui.QCheckBox("", self)

        self.stealth = QtGui.QCheckBox("", self)

        self.safe = QtGui.QCheckBox("", self)

        load_menu = main_menu.addMenu("&File")
        load_menu.addAction(self.img_load)
        load_menu.addAction(self.app_quit)

        launch_button = QtGui.QPushButton("Launch MORRF", self)
        launch_button.resize(250, 40)
        launch_button.move(0, 160)

        layout = QtGui.QVBoxLayout()
        inner_layout = QtGui.QFormLayout()

        inner_layout.addRow("Quickly", self.quick)
        inner_layout.addRow("Stealthy", self.stealth)
        inner_layout.addRow("Safely", self.safe)

        layout.addLayout(inner_layout)
        layout.addWidget(launch_button)

        self.win = QtGui.QWidget(self)
        self.win.setLayout(layout)

        self.setCentralWidget(self.win)

        self.show()

    def load_image(self):
        self.image_name = QtGui.QFileDialog.getOpenFileName(self, "Load Image")
        self.image_window = Sketch(self.image_name)
