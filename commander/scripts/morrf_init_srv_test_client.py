#!/usr/bin/env python

import sys
import rospy

from commander.srv import *
from commander.msg import *

def morrf_init_client():
    
    send_to_server = morrf_init()

    start = coordinate()
    goal = coordinate()

    start.x = input("Enter start x position ")
    start.y = input("\nEnter start y position ")
    goal.x = input("\nEnter goal x position ")
    goal.y = input("\nEnter goal y position ")

    send_to_server.goal = goal
    send_to_server.start = start

    print "Goal x, y is %s, %s" %(send_to_server.goal.x, send_to_server.goal.y)
    print "Start x, y is %s, %s" %(send_to_server.start.x, send_to_server.start.y)

    send_to_server.iterations = input("\nEnter # of iterations ")
    send_to_server.segment_length = input("\nEnter segment length ")
    send_to_server.number_of_trees = input("\nEnter number of trees ")
    send_to_server.objective_number = input("\nEnter the number of objectives " )
    send_to_server.minimum_distance_enabled = input("\nEnter whether minimum distance is enabled ")

    rospy.wait_for_service('morrf_init')

    try:
        morrf_result = rospy.ServiceProxy('morrf_init', morrf)
        response = morrf_result(send_to_server)

        for num in range(len(response.paths)):
            print "Coordinate x: %s y: %s " %(response.paths[num].waypoints[0].x, response.paths[num].waypoints[0].y)

        return response.paths

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    morrf_init_client()
    print "Morrf completed" 


