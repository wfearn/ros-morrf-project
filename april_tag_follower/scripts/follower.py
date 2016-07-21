#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from morrf_ros.msg import multi_objective_path

import math
#import the msg file - April_tag_Pos

class Follower():
    def __init__(self):
        print "Follower started"

        rospy.init_node("follower", anonymous=False)

        self.robot_movement = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

        self.robot_position = rospy.Subscriber("robot_pos", Pose2D, self.follow)
        self.path_getter = rospy.Subscriber("follower", multi_objective_path, self.start_follow)

        self.initialize = False

        self.lin_vel = 1
        self.ang_vel = 1

        self.rate = rospy.Rate(5)

        self.index = 0

        rospy.spin()

    def start_follow(self, path):
        print "Received paths, setting initializer to true"

        self.path = path.waypoints
        self.initialize = True

    def follow(self, pos):
        print "Moving robot to follow path"

        if self.initialize == True:

            if self.index != (len(self.path) - 1):

                if self.dist(pos, self.path[self.index]) > 1:

                    rx = pos.x
                    ry = pos.y
                    rtheta = pos.theta

                    p2 = self.path[self.index]

                    self.new_theta = math.atan2( (p2.x - rx), (p2.y - ry) )

                    if abs(rtheta - new_theta) > 0.01:
                        self.update_orientation(pos)

                    else:
                        self.update_position(pos)

                else:
                    self.index += 1

    def update_position(self, rob_pos):
        print "Updating robot linear position"

        rob_move = Twist()
        rob_move.linear.x = lin_vel

        self.robot_movement.publish(rob_move)


    def update_orientation(self, rob_pos):
        print "Updating robot angular position"
        rob_theta = rob_pos.theta

        rob_turn = Twist()

        if rob_theta - self.new_theta > 0:
            rob_turn.angular.z = 0 - self.ang_vel
        else:
            rob_turn.angular.z = 0 + self.ang_vel

        self.robot_movement.publish(rob_turn)

    def dist(p1, p2):
        return math.sqrt( (p2.x - p1.x)**2 + (p2.y - p2.y)**2 )
