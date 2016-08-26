#!/usr/bin/env python

import sys
import rospy
from morrf_ros.srv import *
from morrf_ros.msg import *

def StartFollowPublisher(path):
    print "Sending path to robot..."

    pub = rospy.Publisher("follower", multi_objective_path, queue_size=10)

    pub.publish(path)


