#!/usr/bin/env python

import sys
from PyQt4 import QtGui, QtCore
from config_window import Config
from image_window import Image

class Gui():

    def __init__(self):
        self.config_window = Config()

