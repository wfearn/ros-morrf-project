#!/usr/bin/env python

import sys
import rospy
from harrt_ros.srv import *
from harrt_ros.msg import *

def StartContinueHarrtPublisher(iterations):
    rospy.wait_for_service("/harrt/refine_paths")
    try:
        harrt_ros = rospy.ServiceProxy("/harrt/refine_paths", harrt_continue)
        response = harrt_ros(iterations)
        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
