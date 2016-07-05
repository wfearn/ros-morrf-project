import sys
import rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *

from image_window import Image

from morrf_ros.msg import *

import cv2 
import numpy as np

from error_popup.not_initialized import NotInitialized
from error_popup.no_image_selected import NoImage

from publishers.costmap_publisher import StartCostmapPublisher
from publishers.commander_publisher import StartCommanderPublisher

STARTX = 1200
STARTY = 1200
WIDTH = 200 
HEIGHT = 300

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

		self.view()
	
	def view(self):
		self.imgLoad = QtGui.QPushButton("Load Image", self)
		self.imgLoad.clicked.connect(self.load_image)		
	
		btn = QtGui.QPushButton("Exit", self)
		btn.clicked.connect(QtCore.QCoreApplication.instance().quit)
	
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
		layout.addWidget(btn)
	
		self.widget = QtGui.QWidget(self)
		self.widget.setLayout(layout)
		self.setCentralWidget(self.widget)

		self.show()	

	def load_image(self):
		self.image_name = QtGui.QFileDialog.getOpenFileName(self, "Load Image")
		self.image_window = Image(self.image_name)
		
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
			#print ("width of desktop: %s", QApplication.desktop().width())	

			self.error1.move((resolution.width() / 2), (resolution.height() / 2))
			#print ("x of error widget: %d, y of error widget: %d" %(self.error1.pos().x(), self.error1.pos().y()))
		
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

			initializer.map = self.map_convert()
			initializer.width = initializer.map.width
			initializer.height = initializer.map.height
		
			initializer.number_of_iterations = 1000
			initializer.segment_length = 5
			initializer.number_of_trees = 5
			initializer.objective_number = self.quick.isChecked() + self.safe.isChecked() + self.stealth.isChecked()
			initializer.minimum_distance_enabled = self.quick.isChecked()
			initializer.method_type = 0 #Weighted Sum		
	
			#print("stealth: ", self.stealth.isChecked())
			self.costmap_response = StartCostmapPublisher(initializer.map, self.stealth.isChecked(), self.safe.isChecked(), self.image_window.getEnemyLocations())
			
			initializer.cost_maps = self.costmap_response.cost_maps
			
			self.morrf_response = StartCommanderPublisher(initializer)

			self.image_window.startPathCycler(self.morrf_response)			
			
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
