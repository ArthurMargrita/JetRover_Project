# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import sleep


DIST_OBJECT_FORWARD = 0.20 #distance minimal à laquelle le lidar va détecter l'objet devant (en m)
DIST_OBJECT_RIGHT = 0.20 #distance minimal à laquelle le lidar va détecter l'objet devant (en m)
DIST_OBJECT_LEFT = 0.20 #distance minimal à laquelle le lidar va détecter l'objet devant (en m)

class LidarSubscriber(object):
	def __init__(self):
		rospy.init_node('lidar_subscriber')
		self.subscription = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		
		self.obstacle_detect_forward = False  # Indicateur pour les obstacles
		self.obstacle_detect_right = False
		self.obstacle_detect_left = False
		
		self.obstacle_detecte = False  # Indicateur pour les obstacles
		self.ranges = []  # Distances relevées par le LiDAR
		self.angle_min = 0.0
		self.angle_max = 0.0
		self.angle_increment = 0.0
		
		rospy.spin()  # Boucle principale ROS
		

	def scan_callback(self, msg):
		self.angle_min = msg.angle_min
		self.angle_max = msg.angle_max
		self.ranges = msg.ranges
		self.angle_increment = msg.angle_increment

		# Appel des fonctions de traitements
		self.traitement_lidar_forward()
		self.traitement_lidar_left()
		self.traitement_lidar_right()
		
		
		if self.obstacle_detect_forward:
			print("objet DEVANT")
		if self.obstacle_detect_right:
			print("objet sur la DROITE")
		if self.obstacle_detect_left:
			print("objet sur la GAUCHE")

	def traitement_lidar_forward(self):
		seuil_distance = DIST_OBJECT_FORWARD 

		# Trouver les indices correspondant à l'angle devant
		indices = []

		for k in range(len(self.ranges)):
			if (self.angle_min + k * self.angle_increment) <= (-5*math.pi/6) or (self.angle_min + k * self.angle_increment) >= (5*math.pi/6):
				indices.append(k)
	
		# Parcourir les indices des distances devant le robot
		self.obstacle_detect_forward = False
		for i in indices:
			#print(i)
			if (0.0 < self.ranges[i]) and ( self.ranges[i] < seuil_distance):
				self.obstacle_detect_forward = True
				break
				
	def traitement_lidar_right(self):
		seuil_distance = DIST_OBJECT_RIGHT 

		indices = []
		for k in range(len(self.ranges)):
			if self.angle_min + k * self.angle_increment <= (5*math.pi)/6 and self.angle_min + k * self.angle_increment >= (math.pi)/2:
				indices.append(k)

		self.obstacle_detect_right = False
		for i in indices:
			if 0.0 < self.ranges[i] < seuil_distance:
				self.obstacle_detect_right = True
				break
				
	def traitement_lidar_left(self):
		seuil_distance = DIST_OBJECT_LEFT 

		indices = []
		for k in range(len(self.ranges)):
			if self.angle_min + k * self.angle_increment <= -math.pi/2 and self.angle_min + k * self.angle_increment >= -5*math.pi/6:
				indices.append(k)

		self.obstacle_detect_left = False
		for i in indices:
			if 0.0 < self.ranges[i] < seuil_distance:
				self.obstacle_detect_left = True
				break

def main():
	
	try:

		print("init")
		lidar = LidarSubscriber()

	except rospy.ROSInterruptException:
		print("Arrêt du programme.")
		
if __name__ == "__main__":
	main()
