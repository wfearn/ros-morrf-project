#!/usr/bin/env python

import rospy
import sys

from commander_gui.config_sketch import SketchConfig

from PyQt4 import QtGui, QtCore


def main():

    app = QtGui.QApplication(sys.argv)

    sk = SketchConfig()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
