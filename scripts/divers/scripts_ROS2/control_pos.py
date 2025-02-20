import rclpy
from rclpy.node import Node
import math
from time import sleep
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class pose_robot_Node(Node):
	def __init__(self):
		super().__init__('control_pos_node')  # Node name
		self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)  # Subscribe to pose topic
		self.message = None
		
		# Variables for robot position
		self.pos_x = None
		self.pos_y = None
		self.pos_theta = None

	def pose_callback(self, msg):
		self.pos_x = msg.x
		self.pos_y = msg.y
		self.pos_theta = msg.theta



def normalize_angle(angle):
    # Normalisation de l'angle pour qu'il soit dans la plage [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))

def calculate_angle(x1, y1, x2, y2):
	delta_x = x1 - x2
	delta_y = y1 - y2
	angle = math.atan2(delta_y, delta_x)
	angle_degrees = math.degrees(angle)
	
	return normalize_angle(angle)
	
def calculate_distance(x1, y1, x2, y2):

    delta_x = x2 - x1
    delta_y = y2 - y1
    distance = math.sqrt(delta_x**2 + delta_y**2)
    
    return distance


def main():
	rclpy.init()
	node = pose_robot_Node()  # instancier la class
	pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Publier le topic 
	while True:
		try:
			# Demander à l'utilisateur de saisir les coordonnées séparées par un espace
			coord_x, coord_y = map(float, input("Entrez les coordonnées (x, y) séparées par un espace : ").split())
			break
		except ValueError:
			print("Les coordonnées doivent être de type float et séparées par un espace.")
		
	msg_twist = Twist()
	
	flag_turn = True
	flag_straight = True
		
	try:
		while rclpy.ok():
			rclpy.spin_once(node)
			while flag_turn:  # Tant que le robot n'est pas en face du point, il tourne
				rclpy.spin_once(node)
				theta = calculate_angle(node.pos_x, node.pos_y, coord_x, coord_y) + math.pi
				
				msg_twist.angular.z = 0.5
				pub.publish(msg_twist)
				

				if (abs(theta - node.pos_theta)) < 0.1:  # Tolérance de 0.1 rad
					flag_turn = False
						
			sleep(2)
			
			while flag_straight:  # Tant que le robot n'est pas en face du point, il tourne
				rclpy.spin_once(node)
				distance = calculate_distance(node.pos_x, node.pos_y, coord_x, coord_y)
				print(distance)
				
				msg_twist.linear.x = 0.5
				msg_twist.angular.z = 0.0
				pub.publish(msg_twist)
				
				print(distance)
				if distance < 0.1:  # Tolérance de 0.01 rad
						flag_straight = False
						
				
									
			

			
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == "__main__":
	main()
