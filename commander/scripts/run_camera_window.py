#!/usr/bin/env python

import rospy, sys

from PyQt4 import QtGui, QtCore

from commander_gui.config_camera import CameraConfig

def main():

    app = QtGui.QApplication(sys.argv)

    c = CameraConfig()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
