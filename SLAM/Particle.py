"""
particle class which will save the pose and the weight of the particle
"""


class Particle:

	def __init__(self,x,y,t,w):

		self.pose = Pose(x,y,t)
		self.weight= w