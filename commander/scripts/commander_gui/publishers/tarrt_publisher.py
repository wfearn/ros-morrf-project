#!/usr/bin/env python

import sys
import rospy
from tarrt_ros.srv import *
from tarrt_ros.msg import *

def StartTarrtPublisher(tarrt_initiator):
    print "Starting TARRT to generate paths..."

    rospy.wait_for_service("/tarrt/get_paths")

    try:
        tarrt_ros = rospy.ServiceProxy("/tarrt/get_paths", tarrt_initialize)

        response = tarrt_ros(tarrt_initiator)

        return response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
