import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import sleep

DIST_OBJECT_FORWARD = 0.20 #distance minimal à laquelle le lidar va détecter l'objet devant (en m)
DIST_OBJECT_RIGHT = 0.20 #distance minimal à laquelle le lidar va détecter l'objet devant (en m)
DIST_OBJECT_LEFT = 0.20 #distance minimal à laquelle le lidar va détecter l'objet devant (en m)

class LidarSubscriber(Node):
	def __init__(self):
		super().__init__('lidar_subscriber')
		self.subscription = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
		
		self.obstacle_detect_forward = False  # Indicateur pour les obstacles
		self.obstacle_detect_right = False
		self.obstacle_detect_left = False
		self.ranges = []  # Distances relevées par le LiDAR
		self.angle_min = 0.0
		self.angle_max = 0.0
		self.angle_increment = 0.0

	def scan_callback(self, msg):
		self.angle_min = msg.angle_min
		self.angle_max = msg.angle_max
		self.ranges = msg.ranges
		self.angle_increment = msg.angle_increment

		# Appel des fonctions de traitements
		self.traitement_lidar_forward()

	def traitement_lidar_forward(self):
		seuil_distance = DIST_OBJECT_FORWARD 

		# Trouver les indices correspondant à l'angle devant
		indices = []
		for k in range(len(self.ranges)):
			if self.angle_min + k * self.angle_increment <= math.pi/6 or self.angle_min + k * self.angle_increment >= (11*math.pi)/6:
				indices.append(k)

		# Parcourir les indices des distances devant le robot
		self.obstacle_detect_forward = False
		for i in indices:
			if 0.0 < self.ranges[i] < seuil_distance:
				self.obstacle_detect_forward = True
				break
				
	def traitement_lidar_right(self):
		seuil_distance = DIST_OBJECT_RIGHT 

		indices = []
		for k in range(len(self.ranges)):
			if self.angle_min + k * self.angle_increment <= math.pi/2 and self.angle_min + k * self.angle_increment >= math.pi/6:
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
			if self.angle_min + k * self.angle_increment <= (11*math.pi)/6 and self.angle_min + k * self.angle_increment >= (3*math.pi)/2:
				indices.append(k)

		self.obstacle_detect_left = False
		for i in indices:
			if 0.0 < self.ranges[i] < seuil_distance:
				self.obstacle_detect_left = True
				break

def main():
	rclpy.init()
	lidar = LidarSubscriber()
	
	try:
		while rclpy.ok():
			rclpy.spin_once(lidar)
			if lidar.obstacle_detect_forward:
				print("objet devant")
			if lidar.obstacle_detect_right:
				print("objet sur la droite")
			if lidar.obstacle_detect_left:
				print("objet sur la gauche")

	except KeyboardInterrupt:
		print("Arrêt du programme.")
	finally:
		lidar.destroy_node()
		rclpy.shutdown()

if __name__ == "__main__":
	print("init")
	main()
