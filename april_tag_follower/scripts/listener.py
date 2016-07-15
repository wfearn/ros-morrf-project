#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose2D
#import the msg file - April_tag_Pos

def follow(data, point):

	for i in range(0, len(data.id)):
		rob_id = data.id[i]
		rob_position = data.pos[i]
		
		v = .1
		ang_vel = .001

		p1 = (point.x, point.y)
		p2 = rob_position

		while dist(p1, p2) > 1:
			theta = math.atan2( (p1[1] - p2[1]), (p1[0] - p2[0]) )

			dx = v * math.cos(theta)
			dy = v * math.sin(theta)	
			
			p2 = (p2[0] + dx, p2[1] + dy)
			data.pos[i].x = p2[0]
			data.pos[i].y = p2[1]


def update_orientation(theta, data):
	rob_orient = data.orientation
			
	while abs(theta - rob_orient) > 0.01:

		if rob_orient - theta > 0:
			rob_orient -= ang_vel 
		else:
			rob_orient += ang_vel

	data.orientation = rob_orient
				

def listener(data, next_point):
	
	rospy.init_node("listener")

	rospy.Subscriber("april_tag_pos", data, next_point, follow)

	rospy.spin()


def dist (p1, p2):
	return math.sqrt( (p2[0] - p1[0]) **2 + (p2[1] - p2[1]) **2 )


