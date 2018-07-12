#!/usr/bin/env python

"""
this code plans the path and drives the robot to the destinations
it takes the input from the localization function and then computes the
Astar path on it.

vishal v kole - vvk3025 - 5/20/2018
Mounika Alluri
Shih-Ting Huang

Go to specified co-ordinates with obstacle avoidance
(Tested on Pioneer P3-DX, ROS Kinetic with Ubuntu 16.04)

"""
import rospy
import time
import math
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from p2os_msgs.msg import MotorState
from mapGUI import *
import Tkinter as tk
from std_msgs.msg import String
from PIL import Image
import matplotlib.image as img, math, heapq as hq
from CustomMap import *
from AstarSearch import *
from Heuristics import *

#satic map constants for the map
WIDTH = 2000
HEIGHT = 700
PARTICLE_SIZE = 10
RESOLUTION = 0.063


PATH = "../catkin_ws/src/project/src/map.png"


#global parameters
pub = None
odom = None
scan = None
coordinates = None
twist_obj = None
hasObstacle = None
obs_loc = None
angle_difference = None
right_done = None
right_done = None
wall_follow = None
sonar = None
sensor_active = None
pub_motor= None
mapr = None
local_done = None
drop_points = 20
wall_safe_dist = 14
path = None
root = tk.Tk()
mapper = Mapper(master=root,height=WIDTH,width=HEIGHT)
#localdata = [10.261235513,12.8677448289, 10.261235513,12.8677448289]
localdata = None


"""
read the coordinate of the destination from the file
"""
def instantiate_fileCoordinates():
	global coordinates
	coordinates = list()
	file = open(str(sys.argv[1]),"r")
	for line in file:
		line_split = line.strip().split()
		coordinates.append([float(line_split[0]),float(line_split[1])])
	
"""
helper function for pixel conversion
"""
def GlobaltoLocal(localX, localY):
		x = [ ((localX-(WIDTH/2))*RESOLUTION) , -1*((localY-(HEIGHT/2))*RESOLUTION)  ]		
		return x

"""
converts the local pixel in the map into the local coordinates of the robot
"""
def pixeltoglobal(path):

	new = list()
	for loc in path:
		new.append(GlobaltoLocal(loc[0],loc[1]))

	return new

"""
convers the pixels in the map into the waypoints on the robot map through 
translation and rotation
"""
def pixelToWaypoints():

	global localdata
	global odom
	global coordinates

	new = list()
	initial = [localdata[0], localdata[1]]
	delta = [odom.pose.pose.position.x, odom.pose.pose.position.y]
	for point in coordinates:
		xold = point[0] -(initial[0])
		yold = point[1] -(initial[1])
		xnew = (xold * math.cos(localdata[2])) + (yold*math.sin(localdata[2]))
		ynew = (yold * math.cos(localdata[2])) - (xold*math.sin(localdata[2]))
		new.append([xnew+delta[0], ynew+delta[1]])
	coordinates = new
	print("coordinates = "+str((coordinates)))
		
"""
drops some points on the path as the pixels are too close for the robot to 
be considered as valid waypoints
"""
def dropWaypoints(path):

	global drop_points

	new = list()
	count = 0
	for item in path:
		count+=1
		if count % drop_points ==0:
			new.append(item)
	return new

"""
adds the waypoints to the destination list which is the final list for the robot to follow
"""
def addToDestinations(path):

	global coordinates

	for items in coordinates:
		path.append(items)
	coordinates = path
	
"""
this function calls the Astar algorithm and gets the pixels in the map
"""
def getPath():

	global localdata
	global path
	global mapper
	global PATH

	cmap = CustomMap(img.imread(PATH))
	heuristics = Heuristics()
	search = Astar(cmap, heuristics)
	start = [localdata[0], localdata[1]]
	start = search.LocaltoGlobal(start[0],start[1])
	destination = [coordinates[0][0], coordinates[0][1]]
	destination = search.LocaltoGlobal(destination[0],destination[1])
	start = [start[0], start[1]]
	destination = [destination[0] , destination[1]]
	path = search.searchPath(start, destination)
	mapper.printpath(path)
	path = pixeltoglobal(path)
	path = dropWaypoints(path)
	addToDestinations(path)
	pixelToWaypoints()
	
"""
instantiate the global parameters to a default value
"""
def instantiate_parameters():

	global twist_obj
	global angular_speed
	global linear_speed
	global hasObstacle
	global angle_difference
	global right_done
	global wall_follow
	global left_done
	global sensor_active
	global mapr
	global local_done


	sensor_active = " "
	wall_follow = False
	right_done = False
	left_done = False
	hasObstacle = False
	local_done = False
	angle_difference = 0
	twist_obj = Twist()

"""
instantiates the publishers
"""
def instantiate_publisher():

	global pub
	global pub_motor

	pub = rospy.Publisher('r1/cmd_vel', Twist, queue_size=10)

"""
sonar callback function which will run the path planner and the safegoto functions
"""
def getSonar(data):

	global sonar
	global localdata
	global mapper
	global path
	global local_done


	sonar = data
	if not localdata is None:
		if local_done is False:
			local_done = True
			getPath()
		walk()
		obstacle_detection()
		maneuver_robot()

"""
instantiates the subscribers
"""
def instantiate_subscribers():
	#sub_scan = rospy.Subscriber('/kinect_laser/scan', LaserScan, callback_scan)
	#rospy.Subscriber('/sonar',SonarArray, getSonar)
	rospy.Subscriber('/r1/sonar',SonarArray, getSonar)
	#rospy.Subscriber("/pose", Odometry, callbackOdom)	
	rospy.Subscriber("/r1/odom", Odometry, callbackOdom)	
	rospy.Subscriber("/r1/localization", String, LocalizationCallback,queue_size=1)

def LocalizationCallback(data):
	"""
	callback for topic published by particle filter
	"""
	global localdata

	print(data)
	data 	  = str(data).strip().split(",")
	
	data[0]   = float(data[0][7:])
	data[1]   = float(data[1])
	data[2]   = float(data[2][:-1])
	localdata = data
	
"""
helper function to publish the speed to the robot
"""
def publish_twist(angular=0,linear=0):

	global twist_obj
	global pub

	twist_obj.angular.z = angular
	twist_obj.linear.x = linear
	pub.publish(twist_obj)

"""
this function drives the robot in case of no obsticles. detects and sets the 
flags if there are obsticles
"""
def walk():

	global coordinates
	global odom
	global hasObstacle
	global angle_difference
	global sonar
	global scan
	global obs_loc
	global right_done
	global wall_follow
	global left_done
	global sensor_active
	global localdata

	angle_threshold = 0.1
	zero = 0.0
	error_dist = 0.2
	
	if not coordinates:
		rospy.loginfo("No Co-ordinates to go. Shutting down!")
		rospy.signal_shutdown("No Co-ordinates to go. Shutting down!")
	
	cur_x = odom.pose.pose.position.x
	cur_y = odom.pose.pose.position.y
	cur_z = odom.pose.pose.orientation.z
	cur_w = odom.pose.pose.orientation.w
		
	cur_theta = 2*math.atan2(cur_z,cur_w)
	#cur_theta += localdata.pose.theta
	if cur_theta> math.pi:
		cur_theta = ((math.pi) + ( math.pi - cur_theta))*-1
	if cur_theta< -1*math.pi:
		cur_theta = ((math.pi) - (( math.pi + cur_theta)*-1))
	
	goal_theta = math.atan2((coordinates[0][1]-cur_y),(coordinates[0][0]-cur_x))
	angle_diff = goal_theta - cur_theta
	dist = math.sqrt((coordinates[0][0] - cur_x)**2 + (coordinates[0][1] - cur_y)**2)  
	angle_difference = angle_diff

	"""
	debugging parameters to analyse the robot
	"""
	print("____________________________________\n" +
	"x "+str(cur_x)+"\n"+
	"y "+str(cur_y)+"\n"+
	"current "+str(cur_theta)+"\n"+
	"goal "+str(goal_theta)+"\n"+
	"angle_diff "+str((angle_diff))+"\n"+
	"dist "+str(dist)+"\n"+
	"Sonar left: " + str(sonar.ranges[0])+"\n"+
	"Sonar right: " + str(sonar.ranges[7])+"\n"+
	"Sonar left center: " + str(sonar.ranges[3])+"\n"+
	"Sonar right center: " + str(sonar.ranges[4])+"\n"+
	"obsloc: " + str(obs_loc)+"\n"+
	"right_done: " + str(right_done)+"\n"+
	"left_done: " + str(left_done)+"\n"+
	"wall_follow: " + str(wall_follow)+"\n"+
	"sensor_active: " + str(sensor_active))
	
	#turning function
	if(math.fabs(angle_diff)>angle_threshold and math.fabs(dist)>error_dist) and not hasObstacle:
		if (angle_diff>0 and math.fabs(angle_diff)<math.pi):
			publish_twist( 0.1,0)
		elif(angle_diff>0 and math.fabs(angle_diff)>=math.pi):
			publish_twist( -0.1,0)		
		elif(angle_diff<0 and math.fabs(angle_diff)<math.pi):
			publish_twist(-0.1,0)
		else:
			publish_twist(0.1,0)			

	elif(math.fabs(dist)>error_dist) and not hasObstacle:
		publish_twist(0,0.25)

	elif not hasObstacle:
		publish_twist(0,0)
		rospy.loginfo("********Reached!**********")
		coordinates.pop(0)

		if not coordinates:
			rospy.loginfo("********* Done, Shutting down **********")
			rospy.signal_shutdown('Shut Down')
				  
"""
odometry callback function
"""
def callbackOdom(data):

	global pub
	global odom
	global pub_motor

	odom = data
	rate = rospy.Rate(10)
	rate.sleep()
	#pub_motor.publish(1)

"""
laser callback function
"""
def callback_scan(data):

	global scan

	scan = data
"""
avoids the obstacles and goes around it
"""
def maneuver_robot():

	global hasObstacle	
	global obs_loc
	global odom
	global angle_difference
	global right_done
	global wall_follow
	global sonar
	global scan
	global left_done
	global sensor_active
	
	dist = 0.3
	
	if hasObstacle and (sonar.ranges[0] >dist) and right_done == False and wall_follow == False and obs_loc == "left":
		publish_twist(-0.1,0.0)
	elif hasObstacle and (sonar.ranges[7] > dist) and left_done == False and wall_follow == False and obs_loc == "right":
		publish_twist(0.1,0.0)
	elif hasObstacle and right_done == False and wall_follow == False:
		publish_twist(0.0,0.0)
		right_done = True
		wall_follow = True
		sensor_active = "left"
	elif hasObstacle and left_done == False and wall_follow == False:
		publish_twist(0.0,0.0)
		left_done = True
		wall_follow = True
		sensor_active = "right"
		

		#if the obstacle is to the left
	if wall_follow == True and sensor_active == "left":	
		if sonar.ranges[0] <dist :
			publish_twist(-0.1,0.1)
		elif sonar.ranges[0] >0.6 :
			publish_twist(0.15,0.15)
		elif sonar.ranges[0] >0.52 :
			publish_twist(0.1,0.1)
		else :
			publish_twist(0.0,0.15)

		#if the obstacle is to the left
	if wall_follow == True and sensor_active == "right":	
		if sonar.ranges[7] <dist:
			publish_twist(0.1,0.1)
		elif sonar.ranges[7] >0.6 :
			publish_twist(-0.15,0.15)
		elif sonar.ranges[7] >0.52 :
			publish_twist(-0.1,0.1)
		else :
			publish_twist(0.0,0.15)
  
	right_thresh = 0.8

		#wall follow to the left
	if sensor_active == "right" and angle_difference <(math.pi/2)- right_thresh and angle_difference >(math.pi/2)+ right_thresh  :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False

		#wall follow to the right
	if sensor_active == "left" and angle_difference <-(math.pi/2)- right_thresh  and angle_difference >-(math.pi/2)+ right_thresh :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False
					
"""
funcion to detect the obstacles in front of the robot
"""	
def obstacle_detection():

	global scan
	global obs_loc
	global hasObstacle
	global sonar
	global right_done
	global wall_follow
	global left_done

	stopping_dist = 0.2

	#stop if free in the world without obstacle
	if ((sonar.ranges[3] > stopping_dist) and (sonar.ranges[2] > stopping_dist) and (sonar.ranges[1] > stopping_dist)
		and ((sonar.ranges[4] > stopping_dist) and (sonar.ranges[5] > stopping_dist) and (sonar.ranges[4] > stopping_dist))) \
			and sonar.ranges[7] >0.3 and sonar.ranges[0] >0.3 :
		hasObstacle = False
		sensor_active = " "
		wall_follow = False
		right_done = False
	

	#Continuously check for obstacles
	if   (sonar.ranges[3] < stopping_dist) or (sonar.ranges[2] < stopping_dist) or (sonar.ranges[1] < stopping_dist):
		hasObstacle = True
		left_done = False
		right_done = False
		wall_follow = False
		obs_loc = "left"

	elif (sonar.ranges[6] < stopping_dist) or (sonar.ranges[5] < stopping_dist) or (sonar.ranges[4] < stopping_dist):
		hasObstacle = True
		right_done = False
		left_done = False
		wall_follow = False
		obs_loc = "right"
	else:
		obs_loc = None
	
"""
main looping function
"""
def main():

	global root

	instantiate_fileCoordinates()
	instantiate_publisher()
	instantiate_parameters()
	rospy.init_node('Explorer', anonymous=True)
	instantiate_subscribers()
	root.mainloop()


if __name__ == '__main__':

	try:
		main()
	except rospy.ROSInterruptException:
		pass









