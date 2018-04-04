#!/usr/bin/python

"""
Mounika Alluri

Occupancy grid generation using laser and sonar data on gazebo

"""
'''
  Some Tkinter/PIL code to pop up a window with a gray-scale
  pixel-editable image, for mapping purposes.  Does not run
  until you fill in a few things.

  Does not do any mapping.

  Z. Butler, 3/2016, updated 3/2018
'''

import Tkinter as tk
from PIL import Image
import ImageTk
import random
import rospy
import math
import sys

from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from nav_msgs.msg import Odometry
from p2os_msgs.msg import MotorState

MAPSIZE = 600
LASERMAX = 8.0
SONARMAX = 4.8
MAPRESOLUTION = 0.05


class Mapper(tk.Frame):

	def __init__(self, *args, **kwargs):
        	tk.Frame.__init__(self, *args, **kwargs)
        	self.master.title("I'm the map!")
        	self.master.minsize(width=MAPSIZE, height=MAPSIZE)

        	# makes a grey-scale image filled with 50% grey pixels
        	self.themap = Image.new("L", (MAPSIZE, MAPSIZE), 128)
        	self.mapimage = ImageTk.PhotoImage(self.themap)

        	# this gives us directly memory access to the image pixels:
        	self.mappix = self.themap.load()
        	# keeping the odds separately saves one step per cell update:
        	self.oddsvals = [[1.0 for _ in range(MAPSIZE)] for _ in range(MAPSIZE)]
        	self.canvas = tk.Canvas(self, width=MAPSIZE, height=MAPSIZE)
        	self.map_on_canvas = self.canvas.create_image(MAPSIZE / 2, MAPSIZE / 2, image=self.mapimage)
        	self.canvas.pack()
        	self.pack()
        	
        	#global variables
        	self.odom= None
        	self.lmsg= None
        	self.smsg= None
        

	def update_image(self):
		"""
		Updates image
		"""
        	self.mapimage = ImageTk.PhotoImage(self.themap)
        	self.canvas.create_image(MAPSIZE / 2, MAPSIZE / 2, image=self.mapimage)

    	def laser_update(self, data):
    		"""
    		laser callback
    		"""
    		self.lmsg= data
    		
    		
    	def sonar_update(self, data):
    		"""
    		sonar callback
    		"""
       		self.smsg = data

        	if sys.argv[1]== "laser":
        		self.laser_map()

        	if sys.argv[1]== "sonar":
        		self.sonar_map()

        	if sys.argv[1]== "both":
	       		self.laser_map()
        		self.sonar_map()

    	def odom_callback(self, data):
    		"""
    		odometry callback
    		"""
        	self.odom= data
	       	
        	
	def sonar_model(self, max, min):
		"""
		piecewise linear function to model sonar sensors
		"""
             	diff = (max-min)/15.0
             	odds = [ 0 for _ in range(30)]
             	for i in range(30):
             		odds[i] = round(max- abs(15-i)* (diff),2)
             	return odds
        	 	
        	
        def sonar_map(self):
        	
        	"""
        	Map generation using sonar data
        	"""
        	# current robot position
        	xr = self.odom.pose.pose.position.x
		yr = self.odom.pose.pose.position.y
		z = self.odom.pose.pose.orientation.z
		w = self.odom.pose.pose.orientation.w
		theta = 2*math.atan2(z,w)
		
		# convert to pixel values and compute bresenhams
        	xr_b  = int((xr/MAPRESOLUTION)+ MAPSIZE/2)
        	yr_b  =  int((-1.0*yr/MAPRESOLUTION)+(MAPSIZE/2))
        	
        	# angles at which sensor sensors are placed relative to center of robot
               	sonar_angs= [math.pi/2, math.pi*5/18, math.pi/6, math.pi/18, \
                              -math.pi/18, -math.pi/6, -math.pi*5/18, -math.pi/2]
		
                for i in range(8):
       
                	dist  = self.smsg.ranges[i]
                	if dist == 4.8 :
        	        	dist = SONARMAX

        	        obs_odds = self.sonar_model(1.5, 1.05)
        	        free_odds = self.sonar_model(0.5, 0.95)
      
        	        w = 30	        
        	        for j in range(w):
        	        
        	                beta  = (j -15)*(math.pi/180.0)
        	        	x_end = xr + dist*math.cos(theta+ sonar_angs[i]+beta)
        	        	y_end = yr + dist*math.sin(theta+ sonar_angs[i]+beta)
        	        	x_end_b =  int((x_end/MAPRESOLUTION)+MAPSIZE/2) 
        	        	y_end_b =   int((-1.0*y_end/MAPRESOLUTION)+MAPSIZE/2)
			        bres_list = list(self.bresenham(xr_b,yr_b, x_end_b, y_end_b))
	                	
	                	# Decrease the odds till the end of bresenham's list
				for item in (bres_list[:-2]):
					map_x = item[0]
					map_y = item[1]
					self.oddsvals[map_x][map_y]*= free_odds[j]
					self.mappix[map_x, map_y] = self.odds_to_pixel_value(self.oddsvals[map_x][map_y])
					
                        	# Increase the odds for the last two values of bresenham's list(obstacle)
				map_x1 = (bres_list[-2])[0]
				map_y1 = (bres_list[-2])[1]
				self.oddsvals[map_x1][map_y1]*= obs_odds[j]
				self.mappix[map_x1, map_y1] = self.odds_to_pixel_value(self.oddsvals[map_x1][map_y1])
			
				map_x = x_end_b
				map_y = y_end_b
				if dist!= SONARMAX:
					self.oddsvals[map_x][map_y]*= obs_odds[j]
				self.mappix[map_x, map_y] = self.odds_to_pixel_value(self.oddsvals[map_x][map_y])
			
        	# this puts the image update on the GUI thread, not ROS thread!
                # also note only one image update per scan, not per map-cell update
               
		self.after(0, self.update_image)
		
    		
    	def laser_map(self):
    		"""
    		Map generation using sonar data
    		"""
    		
        	angle_increment = self.lmsg.angle_increment
        	n = len(self.lmsg.ranges)
        	
        	#current robot position
        	xr = self.odom.pose.pose.position.x
		yr = self.odom.pose.pose.position.y
		z = self.odom.pose.pose.orientation.z
		w = self.odom.pose.pose.orientation.w
		theta = 2*math.atan2(z,w)
		
		# convert to pixel cordinates
		xr_b  = int((xr/ MAPRESOLUTION)+ MAPSIZE/2)
        	yr_b  =  int((-1.0*yr/ MAPRESOLUTION)+(MAPSIZE/2))


                for i in range(0, n,20):
       
               		alpha = (i-320)* angle_increment
                	dist  = self.lmsg.ranges[i]
                	if dist == float('inf') :
        	        	dist = LASERMAX
        	        x_end = xr + dist*math.cos(theta+alpha)
        	        y_end = yr + dist*math.sin(theta+alpha)
        	        
        	        # convert to pixel values and compute bresenhams     
        	        x_end_b =  int((x_end/MAPRESOLUTION)+MAPSIZE/2)
        	        y_end_b =   int((-1.0*y_end/MAPRESOLUTION)+MAPSIZE/2)
			bres_list = list(self.bresenham(xr_b,yr_b, x_end_b, y_end_b))
			
			# Decrease the odds till the end of bresenham's list
			for item in bres_list[:-2]:
				map_x = item[0]
				map_y = item[1]
				self.oddsvals[map_x][map_y]*= 0.5
				self.mappix[map_x, map_y] = self.odds_to_pixel_value(self.oddsvals[map_x][map_y])

			# Increase the odds for the last two values of bresenham's list(obstacle)
			map_x1 = (bres_list[-2])[0]
			map_y1 = (bres_list[-2])[1]
			self.oddsvals[map_x1][map_y1]*= 1.5
			self.mappix[map_x1, map_y1] = self.odds_to_pixel_value(self.oddsvals[map_x1][map_y1])
			
			map_x2 = x_end_b
			map_y2 = y_end_b
			if dist!= LASERMAX:
				self.oddsvals[map_x2][map_y2]*= 1.5
			self.mappix[map_x2, map_y2] = self.odds_to_pixel_value(self.oddsvals[map_x2][map_y2])
        	                  
        	# this puts the image update on the GUI thread, not ROS thread!
                # also note only one image update per scan, not per map-cell update
              
		self.after(0, self.update_image)
		
	
	def bresenham(self,x0, y0, x1, y1):
    		"""
    		Takes integer inputs
    		returns integer coordinates on the line from (x0, y0) to (x1, y1) inclusive of (x0,y0) and (x1,y1)
    		"""
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
      		
		
    	def odds_to_pixel_value(self, odds):
    		"""
    		function that maps from odds[0,inf) --> pixels[0,255]
    		"""
                                                                                                                                                  
        	prob = odds / (1.0 + odds)
        	pixel_value = int((1.0-prob) *255)
        
        	return pixel_value
   

def main():

    print("Enter '1' for laser and '2' for sonar and '3' for laser+sonar")
    rospy.init_node("mapper")
    root = tk.Tk()
    m = Mapper(master=root, height=MAPSIZE, width=MAPSIZE)
    
    #suscribers
    rospy.Subscriber("/scan", LaserScan, m.laser_update,queue_size=1)
    rospy.Subscriber("/pose",Odometry, m.odom_callback, queue_size=1)
    rospy.Subscriber("/sonar", SonarArray, m.sonar_update,queue_size=1)
    root.mainloop()


if __name__ == "__main__":
    main()
