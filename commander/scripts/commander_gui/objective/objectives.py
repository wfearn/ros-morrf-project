#!/usr/bin/env python

from objective import Objective

class Objectives:
    def __init__(self):
        self.objs = []

    def placeObjective(self, objective, x, y):
        if objective != "enemy":
            if not self.contains(objective):

                o = Objective(objective, x, y)
                self.objs.append(o)

            else:
                self.getObjective(objective).setPosition(x, y)

        else:
            o = Objective(objective, x, y)
            self.objs.append(o)

    def contains(self, objective_type):
        for obj in self.objs:
            if objective_type == obj.getObjectiveType():
                return True

        return False

    def updateObjectivesFromClick(self, x, y):
        for obj in self.objs:
            if obj.isInsideBoundary(x, y):

                obj.setClicked(True)

    def updateObjectivesFromMove(self, x, y):
        for obj in self.objs:
            if obj.isClicked():

                obj.setPosition(x, y)

    def updateObjectivesFromRelease(self, x, y):
        for obj in self.objs:
            if obj.isClicked():
                obj.setClicked(False)

    def getObjective(self, objective):

        if self.contains(objective):
            for obj in self.objs:

                if obj.getObjectiveType() == objective:
                    return obj

        else:
            return None

    def reset(self):
        self.objs = []
