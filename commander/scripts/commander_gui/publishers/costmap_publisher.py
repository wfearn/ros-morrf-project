#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *
from commander.srv import *

def StartCostmapPublisher(map_image, stealth, safe, enemy_points):
    print "Received at costmap publisher, %s" % map_image.name
    rospy.wait_for_service("/morrf/get_cost_map")
    try:
        boundary_img = rospy.ServiceProxy("/morrf/get_cost_map", get_cost_map)

        response = boundary_img(map_image, stealth, safe, enemy_points)

        print "Received costmap array, %s" % str(response.cost_maps)

        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
