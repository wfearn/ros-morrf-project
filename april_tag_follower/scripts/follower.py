#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from morrf_ros.msg import multi_objective_path

import math
#import the msg file - April_tag_Pos

DISTANCE_THRESHOLD = 20

#Corresponds to about 5 degrees
THETA_THRESHOLD = .087

class Follower():
    def __init__(self):
        rospy.init_node("follower", anonymous=False)

        self.robot_movement = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

        self.robot_position = rospy.Subscriber("robot_pos", Pose2D, self.follow)
        self.path_getter = rospy.Subscriber("follower", multi_objective_path, self.start_follow)

        self.initialize = False

        self.lin_vel = .25
        self.ang_scalar = .50

        self.rate = rospy.Rate(5)

        self.index = 0

        rospy.spin()

    def start_follow(self, path):

        self.path = path.waypoints
        self.initialize = True

    def follow(self, robot_pos):

        if self.initialize == True:
            print "Moving robot..."

            if self.index != (len(self.path) - 1):

                if self.dist(robot_pos, self.path[self.index]) > DISTANCE_THRESHOLD:
                    rx = robot_pos.x
                    ry = robot_pos.y
                    rtheta = robot_pos.theta

                    waypoint = self.path[self.index]
                    self.angle_to_waypoint = math.atan2( (waypoint.y - ry), (waypoint.x - rx) )

                    if is_negative(rtheta):

                        #Converts it positive
                        rtheta += 2 * math.pi

                        robot_pos.theta = rtheta

                    if is_negative(self.angle_to_waypoint):
                        self.angle_to_waypoint += 2 * math.pi

                    rob_move = Twist()
                    rob_move.angular.z = 0
                    if abs(rtheta - self.angle_to_waypoint) > THETA_THRESHOLD:
                       rob_move.angular.z = self.update_orientation(robot_pos)

                    else:
                        rob_move.linear.x = self.lin_vel

                    self.robot_movement.publish(rob_move)

                else:
                    self.index += 1

            else:
              self.initialize = False

    def update_position(self, rtheta, angular_vel):
        if abs(rtheta - self.angle_to_waypoint) > 0.174:
            return 0
        else:
            return -( ( self.lin_vel / ( math.pi / self.ang_scalar ) ) * abs( angular_vel ) ) + self.lin_vel

    def update_orientation(self, rob_pos):
        rob_theta = rob_pos.theta
        diff = rob_theta - self.angle_to_waypoint

        if abs(diff) > math.pi:
            diff = diff - (sign(diff) * (2 * math.pi))

        return  sign(diff) * self.ang_scalar * max(abs(diff), 0.5)



    def dist(self, p1, p2):
        return math.sqrt( (p2.x - p1.x)**2 + (p2.y - p1.y)**2 )

def sign(number):
    if number >= 0:
        return 1
    else:
        return -1

def is_negative(num):
    return num < 0


