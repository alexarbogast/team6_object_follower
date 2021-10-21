import numpy as np
from enum import Enum

class State(Enum):
	GoToGoal = 1
	AvoidObstacle = 2

class Waypoint:
	def __init__(self, x=0, y=0):
		self.x, self.y = x, y

	def __str__(self):
		return "({0}, {1})".format(self.x, self.y)

	# get waypoint heading in radians
	def GetHeading(self):
		return np.arctan2(self.y, self.x)

	def GetDistance(self):
		return np.sqrt(self.x*self.x + self.y*self.y)

	def Scale(self, percentage):
		self.x += percentage*self.x
		self.y += percentage*self.y
		return self