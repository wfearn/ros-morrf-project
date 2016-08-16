#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *

def StartCommanderPublisher(morrf_initiator):
    print "Starting MORRF to generate paths..."

    rospy.wait_for_service("/morrf/get_multi_obj_paths")
    pub = rospy.Publisher("morrf_response", multi_path_array, queue_size = 10)

    try:
        morrf_ros = rospy.ServiceProxy("/morrf/get_multi_obj_paths", morrf_initialize)

        response = morrf_ros(morrf_initiator)

        #pub.publish(response.path_array)

        return response.path_array

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
