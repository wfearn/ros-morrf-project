#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *

def StartContinuePublisher(iterations):
    print "Continuing MORRF with %s iterations" % iterations
    rospy.wait_for_service("/morrf/continue")
    try:
        morrf_ros = rospy.ServiceProxy("/morrf/continue", morrf_continue)
        response = morrf_ros(iterations)
        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
