#!/usr/bin/env python

import rospy, sys

from PyQt4 import QtGui, QtCore

from Xlib import *

import pygtk, gtk, gobject

from commander_gui.config_camera import CameraConfig

def main():

    gtk.gdk.threads_init()

    app = QtGui.QApplication(sys.argv)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_X11InitThreads)

    c = CameraConfig()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
