#!/usr/bin/env python

import rospy, sys

from PyQt4 import QtGui, QtCore

from commander_gui.turtlebot_config import TurtlebotConfig

def main():

    app = QtGui.QApplication(sys.argv)

    c = TurtlebotConfig()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
