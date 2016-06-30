#!/usr/bin/env python

import rospy

import sys
from PyQt4 import QtGui, QtCore
from commander_gui.gui import Gui


def main():

    app = QtGui.QApplication(sys.argv)

    GUI = Gui()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
