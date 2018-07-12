"""
this function will find the heuristic cost for the given pixel
"""
import math


class Heuristics:

	def getHeuristicCost(self, current, destination):

		return math.sqrt(math.pow((current[0] - destination[0]), 2) +
						 math.pow((current[1] - destination[1]), 2))

