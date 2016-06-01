#!/usr/bin/env python
import sys
from PyQt4 import QtGui, QtCore
from gui import Gui
from config_window import Config

def main():
    
    app = QtGui.QApplication(sys.argv)

    #image_name = raw_input("Input image name: ")

    GUI = Gui()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
