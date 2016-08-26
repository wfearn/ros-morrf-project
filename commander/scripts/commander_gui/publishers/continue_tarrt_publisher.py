#!/usr/bin/env python

import sys
import rospy
from tarrt_ros.srv import *
from tarrt_ros.msg import *

def StartContinueTarrtPublisher(iterations):
    rospy.wait_for_service("/tarrt/refine_paths")
    try:
        tarrt_ros = rospy.ServiceProxy("/tarrt/refine_paths", tarrt_continue)
        response = tarrt_ros(iterations)
        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
