#!/usr/bin/env python


from commander.srv import *
from commander.msg import *
import rospy

start = coordinate()
start.x = 30
start.y = 40

goal = coordinate()
goal.x = 50
goal.y = 60

response = morrf_path()
response.waypoints.append(start)
response.waypoints.append(goal)


def handle_morrf_init(request):

    print "Received morrf init"
    print "Goal at %s,%s" % (request.init.goal.x, request.init.goal.y)
    print "Start at %s,%s" % (request.init.start.x, request.init.start.y)
    print "Iterations are %s" % request.init.iterations
    print "Segment length is %s" % request.init.segment_length
    print "number of trees is %s" % request.init.number_of_trees
    print "Objective number is %s" % request.init.objective_number
    print "min distance enabled is %s" % request.init.minimum_distance_enabled

    send_back = morrfResponse()
    send_back.paths.append(response)

    return send_back


def morrf_init_server():
    rospy.init_node('morrf_init_server')
    s = rospy.Service('morrf_init', morrf, handle_morrf_init)
    print "Ready to take a morrf_init variable"
    rospy.spin()

if __name__ == "__main__":
    morrf_init_server()
