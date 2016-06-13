#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *
from commander.srv import *

def StartImagePublisher(map_image):
    print "Received at commander publisher, %s" % map_image.name
    rospy.wait_for_service("/morrf/get_boundary_image")
    try:
        boundary_img = rospy.ServiceProxy("/morrf/get_boundary_image", test_boundary_img)
        response = morrf_ros(map_image)
        print "Received boundary image, %s" % str(response)
        #print image

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
