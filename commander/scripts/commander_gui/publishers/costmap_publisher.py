#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *
from commander.srv import *
from commander.msg import *

def StartCostmapPublisher(map_image, stealth, safe, enemy_points):
    print "Generating costmaps..."

    rospy.wait_for_service("/morrf/get_cost_map")
    pub = rospy.Publisher("costmap_response", costmap_response, queue_size = 10)

    try:
        costmaps = rospy.ServiceProxy("/morrf/get_cost_map", get_cost_map)

        response = costmaps(map_image, stealth, safe, enemy_points)

        return response.response


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
