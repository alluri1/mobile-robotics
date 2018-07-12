#!/usr/bin/python
"""
  Some Tkinter/PIL code to pop up a window with a gray-scale
  pixel-editable image, for mapping purposes.

  Z. Butler, 3/2016, updated 3/2018
  vishal v kole - vvk3025 - 5/20/2018
  Mounika Alluri
  Shih-Ting Huang

"""

import Tkinter as tk
from PIL import Image
import ImageTk
import random
import rospy, math
from sensor_msgs.msg import LaserScan

#map global parameters to scale with the image
WIDTH = 2000
HEIGHT = 700
PARTICLE_SIZE = 10
RESOLUTION = 0.063


PATH = "../catkin_ws/src/project/src/map.png"


class Mapper(tk.Frame):
	"""
	displayes the map and computes on the map. provides the distance of the wall
	in the image
	"""

	def __init__(self, *args, **kwargs):
		global PATH
		tk.Frame.__init__(self, *args, **kwargs)
		self.master.title("I'm the map!")
		self.master.minsize(width=WIDTH,height=HEIGHT)

		# # makes a grey-scaleimage filled with 50% grey pixels
		# self.themap = Image.new("L",(MAPSIZE,MAPSIZE),128)

		# read project map image
		self.themap = Image.open(PATH).convert("RGB")
		self.mapimage = ImageTk.PhotoImage(self.themap)

		# this gives us directly memory access to the image pixels:
		self.mappix = self.themap.load()
		# keeping the odds separately saves one step per cell update:
		self.oddsvals = [[1.0 for _ in range(WIDTH)] for _ in range(HEIGHT)]

		self.canvas = tk.Canvas(self,width=WIDTH, height=HEIGHT)
		self.last_particles = list()
		self.map_on_canvas = self.canvas.create_image(WIDTH/2, HEIGHT/2, image = self.mapimage)
		self.canvas.pack()
		self.pack()

	"""
	updates the image with the new matrix information
	"""
	def update_image(self):

		self.mapimage = ImageTk.PhotoImage(self.themap)
		self.canvas.create_image(WIDTH/2, HEIGHT/2, image = self.mapimage)
		
	"""
	converts the map point into a pixel coordinate in the image
	"""
	def LocaltoGlobal(self,localX, localY):
		x = [int((localX/RESOLUTION)+(WIDTH/2)), int((-1*(localY/RESOLUTION))+(HEIGHT/2))]
		return x
		
	"""
	prints the Astar path on the map
	"""
	def printpathAstar(self, loc_list):

		for point in loc_list:
			self.mappix[point[1][0] , point[1][1]] = (255, 0,0)
		self.after(0,self.update_image)
	
	"""
	Prints path on the map with just the pixel coordinate values.
	"""
	def printpath(self, loc_list):

		for point in loc_list:
			self.mappix[point[0] , point[1]] = (255, 0,0)
		self.after(0,self.update_image)
		
	"""
	prints all the particles provided to the function and displays the map
	"""
	def particle_update(self, particles):
		
		if self.last_particles:
			for point in self.last_particles:
				self.mappix[point[0],point[1]] = (255, 255,255)
		
		self.last_particles = list()
		
		for point in particles:
			[x, y] = self.LocaltoGlobal(point.pose.x, point.pose.y)
			self.mappix[x , y] = (255, 0,0)
			self.last_particles.append([x , y])
		self.after(0,self.update_image)

	#computes the X and Y coordinates for an angle and distance from the center
	def angleToXY(self, theta, dist):

		x = dist * math.cos(theta)
		y = dist * math.sin(theta)
		return [x, y]

	#source credits for the function to the creator at https://pypi.python.org/pypi/bresenham/0.2
	#Copyright 1990-2018, Python Software Foundation
	def bresenham(self,x0, y0, x1, y1):

		dx = x1 - x0
		dy = y1 - y0
		
		xsign = 1 if dx > 0 else -1
		ysign = 1 if dy > 0 else -1
		
		dx = abs(dx)
		dy = abs(dy)
		
		if dx > dy:
			xx, xy, yx, yy = xsign, 0, 0, ysign
	
		else:
			dx, dy = dy, dx
			xx, xy, yx, yy = 0, ysign, xsign, 0
		D = 2*dy - dx
		y = 0

		for x in range(dx + 1):
			yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
			if D >= 0:
				y += 1
				D -= 2*dx
			D += 2*dy
	
	"""
	returns the pixel value
	"""
	def getValue(self, x, y):

		return self.mappix[x , y]

	"""
	returns the euclidean distance of the two points
	"""
	def getDistance(self,x1,y1,x2,y2):

		return math.sqrt(math.pow((x1-x2),2)+math.pow((y1-y2),2))
		
	"""
	checks if the pixel is in the map(the white part)
	"""
	def isPixelInMap(self, xv, yv):

		point = self.LocaltoGlobal(xv, yv)
		if (self.mappix[point[0], point[1]][0] != 0 or self.mappix[point[0], point[1]][1] != 0
				or self.mappix[point[0], point[1]][2] != 0):
			return True
		return False

	"""
	returns the left, center and the right distance of the local pixel(as provided in the map from the website)
	"""
	def getReading(self, x, y, t):

		endpoints = []
		[x, y] = self.LocaltoGlobal(x, y)
		
		for angle in [t-(math.pi/2),t,t+(math.pi/2)]:	
			[tx, ty] = self.angleToXY(angle,(8/ RESOLUTION))
			locx = x+int(math.floor(tx))
			locy = y+int(math.floor(ty))
			allpoints = (list(self.bresenham(x,y,locx, locy)))
			flag = False
			for point in allpoints:
				if point[0] >=0 and point[0] <=2000 and point[1] >=0 and point[1] <=700 :   
					if self.mappix[point[0], point[1]][0] == 0 and self.mappix[point[0], point[1]][1] == 0 \
							and self.mappix[point[0], point[1]][2] == 0:
						flag = True
						endpoints.append(self.getDistance(x,y,point[0],point[1])*RESOLUTION)
						break

			if flag == False:
				endpoints.append(self.getDistance(x,y,allpoints[-1][0],allpoints[-1][1])*RESOLUTION)
		
		endpoints.reverse()

		#adding robot width calibration
		endpoints[0] -= 0.308
		endpoints[2] -= 0.187
				
		return endpoints


def main():

	rospy.init_node("mapper")

	root = tk.Tk()
	m = Mapper(master=root,height=WIDTH,width=HEIGHT)

	for i in range(PARTICLE_SIZE):
		# random Initialize particle location for testing
		p = Particle()
		p.pose.xp = random.randint(100,1900)
		p.pose.yp = random.randint(100,600)
		m.particle_update(p)

	root.mainloop()

if __name__ == "__main__":
	main()
