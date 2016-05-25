#!/usr/bin/env python

from commander.msg import *
from commander.srv import *
import rospy
from controller import Controller


def main():
    cord = coordinate()
    cord.x = 20
    cord.y = 30

    print "Coordinate \nx: %s\ny: %s\n" %(cord.x, cord.y)
    print "Creating morrf path\n"

    path = morrf_path()

    negative = 6
    path.waypoints.append(cord)

    print "Path coordinate \nx: %s\ny:%s"%(path.waypoints[0].x, path.waypoints[0].y)
    path.costs.append(256)
    path.costs.append(300)
    path.costs.append(500)

    print "Printing path cost values\n"

    for num in range(len(path.costs)):
        print str(path.costs[num])


    print "Testing controller initialize method\n"
    con = Controller()
    con.load_parameters_from_user_input()

if __name__ == "__main__":
    main()
