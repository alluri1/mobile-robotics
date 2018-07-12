"""
Astar algorithm which will compute the shortest path to the destination 
"""
import heapq as hq

class Astar:

	def __init__(self, fmap, heuristics):

		self.mapr = fmap
		self.heuristics = heuristics
		self.heap = []
		self.trace = []
	
	"""
	convert local coordinate to the pixel coordinate
	"""
	def LocaltoGlobal(self,localX, localY):

		x = [int((localX/RESOLUTION)+(WIDTH/2)), int((-1*(localY/RESOLUTION))+(HEIGHT/2))]
		return x
	
	"""
	checks if the child is considered or not
	"""
	def childIn(self, child, child_list):

		for tuples in child_list:
			if tuples[1][0] == child[0] and tuples[1][1] == child[1]:
				return True
		return False
		
	"""
	generates a valid starting point if the localised point is too close
	to the wall
	"""
	def getValidStart(self, start):
		global mapper

		print("start = "+str(start))
		for i in [1,2,3,4,5]:
			for j in range(-i,i+1,1):
				for k in range(-i,i+1,1):
					if self.mapr.validate(mapper, (start[0]+j) ,(start[1]+k)):
						print("valid start = "+str([(start[0]+j) ,(start[1]+k)]))
						return [(start[0]+j) ,(start[1]+k)]

	"""
	searches the path in the map and returns the pixels in the path
	"""
	def searchPath(self, start, destination):
		global mapper

		start = self.getValidStart(start)
		hq.heappush(self.heap, (999, start))
		while self.heap:
			mapper.printpathAstar(self.heap)
			current_element = hq.heappop(self.heap)
			self.trace.append(current_element)
			if (current_element[1][0] == destination[0] and current_element[1][1] == destination[1]) \
					or (float(current_element[0])<10.0):
				ret = []
				for items in self.trace:
					ret.append(items[1])
				self.trace = list()
				self.heap = list()
				return ret
			else:
				childrens = self.mapr.getChildren(current_element[1])
				for childs in childrens:
					if not self.childIn(childs, self.heap) and not self.childIn(childs, self.trace):
						hq.heappush(self.heap,
									(self.heuristics.getHeuristicCost(childs, destination), childs))

