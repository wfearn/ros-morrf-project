#!/usr/bin/env python


from PIL import Image
import numpy as np
import os, json, rospy, sys
from commander.srv import *
from commander.msg import *
FILE = "output.json"
IMAGE_FILE_PATH = "/home/wfearn/catkin_ws/src/commander/data/{}"

class Controller:

	def __init__(self):
	        self.morrf_params = morrf_init()	

        def load_parameters_from_user_input(self):
            self.morrf_params.goal.x = input("\nEnter goal x position ")
            self.morrf_params.goal.y = input("\nEnter goal y position ")
            self.morrf_params.start.x = input("\nEnter start x position ")
            self.morrf_params.start.y = input("\nEnter start y position ")

            self.morrf_params.iterations = input("\nEnter number of iterations ")
            self.morrf_params.segment_length = input("\nEnter segment length ")
            self.morrf_params.number_of_trees = input("\nEnter number of trees ")
            self.morrf_params.objective_number = input("\nEnter number of objectives ")
            self.morrf_params.minimum_distance_enabled = input("\nEnter if minimum distance is enabled ")
            
            image_name = raw_input("\nEnter name of image to load ")
            self.load_image(image_name)

            print "Verifying..."
            print "\nGoal is at %s,%s\nStart is at %s,%s\nIterations are %s\nnumber of trees is %s\nsegment length is %s\nnumber of objectives is %s\nminimum distance enabled is %s" %(self.morrf_params.goal.x, self.morrf_params.goal.y, self.morrf_params.start.x, self.morrf_params.start.y, self.morrf_params.iterations, self.morrf_params.number_of_trees, self.morrf_params.segment_length, self.morrf_params.objective_number, self.morrf_params.minimum_distance_enabled)

	def load_parameters_from_folder(self, filepath):
            os.chdir(filepath)
            try:
                output = json.load(open(FILE))

                self.morrf_params.goal.x = output["goalx"]
                self.moorf_params.goal.y = output["goaly"]
                self.morrf_params.start.x = output["startx"]
                self.morrf_params.start.y = output["starty"]

                self.morrf_params.iterations = output["iterations"]
                self.morrf_params.segment_length = output["segment_length"]
                self.morrf_params.number_of_trees = output["number_of_trees"]
                self.morrf_params.objective_number = output["objective_number"]
                self.moorf_params.minimum_distance_enabled = output["min_dist_enabled"]

                self.load_image(output["map"])
                self.load_image_directory(output["cost_maps"])

            except:
                raise IOError("File not found")

	def load_image(self, image_name):
            im = Image.open(IMAGE_FILE_PATH.format(image_name))  
            pix = im.load()

            cols, rows = im.size
            
            #Initialize image.msg datatype

            morrf_image = image()
            morrf_image.name = image_name
            morrf_image.width = cols
            morrf_image.height = rows

            for i in range(cols):
                pixel_array = pixel1d()
                for j in range(rows):
                    p = pixel()

                    #Indexing is reversed because matrix is column major

                    p.r, p.g, p.b, p.a =  pix[i, j]
                    pixel_array.pixels.append(p)
                    print str(len(pixel_array.pixels))
                    
                morrf_image.pixels.append(pixel_array) 

            self.morrf_params.map = morrf_image

        def get_cost_map_images(self, filepath_to_image, enemy_locations, start, goal):
            print "Getting cost map images"
