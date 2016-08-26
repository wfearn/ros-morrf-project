#!/usr/bin/env python

import sys
import rospy
from harrt_ros.srv import *
from harrt_ros.msg import *

def StartHarrtPublisher(harrt_initiator):
    print "Starting harrt to generate paths..."

    rospy.wait_for_service("/harrt/get_paths")

    try:
        harrt_ros = rospy.ServiceProxy("/harrt/get_paths", harrt_initialize)

        response = harrt_ros(harrt_initiator)

        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
