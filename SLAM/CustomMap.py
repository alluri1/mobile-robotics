"""
crates a custom map of the image and the functions to work with
"""


class CustomMap:

	def __init__(self, fmap):

		self.mapr = fmap
	
	"""
	validates if the pixel is not too close to the map
	"""
	def validate(self, themap, x, y):

		global wall_safe_dist

		diff = wall_safe_dist
		if themap.getValue(x+diff, y+diff) != (0,0,0) and themap.getValue(x+diff, y) != (0,0,0) and themap.getValue(x+diff, y-diff) != (0,0,0) and themap.getValue(x, y-diff) != (0,0,0) and themap.getValue(x-diff, y-diff) != (0,0,0) and themap.getValue(x-diff, y) != (0,0,0) and themap.getValue(x-diff, y+diff) != (0,0,0) and themap.getValue(x, y+diff) != (0,0,0):
			return True
		return False
		
	"""
	gets the children of the current pixel which are valid pixels considering
	the safe distance from the wall
	"""
	def getChildren(self, point):

		global mapper

		childrens = []

		for i in [-1, +1, 0]:
			for j in [-1, +1, 0]:
				if self.validate(mapper, (point[0]+i) ,(point[1]+j)):
					childrens.append([point[0] + i, point[1] + j])
		childrens.pop()
		return childrens
