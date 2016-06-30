#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *

def StartContinuePublisher(boolean, iterations):
    print "Starting continue publisher"
    rospy.wait_for_service("/morrf/continue")
    try:
        morrf_ros = rospy.ServiceProxy("/morrf/continue", continuation)
        response = morrf_ros(boolean, iterations)
        print "Received continued paths, %s" % str(response)
        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
