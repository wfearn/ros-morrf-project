#!/usr/bin/env python

class Coordinate(object):

    def __init__(self, xpos, ypos):
        self.x = xpos
        self.y = ypos

    def __str__(self):
        return "\nx: %s\ny: %s" % (str(self.x), str(self.y))

    def get_point(self):
        return (self.x, self.y)

    def set_x(self, xpos):
        self.x = xpos

    def set_y(self, ypos):
        self.y = ypos
