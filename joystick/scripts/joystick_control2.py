#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVel:
	def __init__(self):
		# Initialisation du nœud et du publisher
		rospy.init_node('joy_to_cmd_vel', anonymous=True)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/joy', Joy, self.joy_callback)

		# Mode de contrôle (True = déplacement, False = bras)
		self.mode = 0

		# Dernière commande valide
		self.last_twist = Twist()

		# Dernières valeurs d’axes mémorisées (évite les retours à zéro intempestifs)
		self.last_axes = [0] * 6  # On suppose que 6 axes sont utilisés

		# Fréquence de publication plus élevée
		self.rate = rospy.Rate(30)  # 30 Hz pour une meilleure fluidité

		# Facteur de rampe d'accélération
		self.accel_factor = 0.1

	def apply_ramp(self, target, current):
		"""Applique une rampe pour lisser les changements de vitesse."""
		return current + self.accel_factor * (target - current)

	def joy_callback(self, data):
		"""Callback pour traiter les données du joystick."""
		self.last_axes = data.axes
		# print(self.last_axes)

		if data.buttons[8] == 1:
			if self.mode == 0:
				rospy.loginfo("Mode changé : Déplacement 1")
				self.mode = 1
				return
			if self.mode == 1:
				rospy.loginfo("Mode changé : Déplacement 2")
				self.mode = 2
				return
			if self.mode == 2:
				rospy.loginfo("Mode changé : bras")
				self.mode = 0
				return

		if self.mode == 0:
			if data.buttons[10] == 1:
				rospy.loginfo("Arrêt d'urgence activé !")
				self.last_twist = Twist()  # Remet toutes les valeurs à zéro
				self.last_axes = [0.0] * len(self.last_axes)  # Réinitialise les axes
				return
				
			if data.buttons[4] == 1:
				target_y = -0.1
			elif data.buttons[5] == 1:
				target_y = 0.1
			else:
				target_y = 0.0

			target_x = self.last_axes[1] / 2.5  # Avant/arrière (joystick gauche)
			target_z = self.last_axes[3] / 2  # Rotation

			self.last_twist.linear.x = self.apply_ramp(target_x, self.last_twist.linear.x)
			self.last_twist.linear.y = self.apply_ramp(target_y, self.last_twist.linear.y)
			self.last_twist.angular.z = self.apply_ramp(target_z, self.last_twist.angular.z)

		if self.mode == 1:
			if data.buttons[10] == 1:
				rospy.loginfo("Arrêt d'urgence activé !")
				self.last_twist = Twist()  # Remet toutes les valeurs à zéro
				self.last_axes = [0.0] * len(self.last_axes)  # Réinitialise les axes
				return
				
			if data.buttons[4] == 1:
				target_y = -0.1
			elif data.buttons[5] == 1:
				target_y = 0.1
			else:
				target_y = 0.0

			target_x = self.last_axes[1] / 6  # Avant/arrière (joystick gauche)
			target_z = self.last_axes[3] / 3  # Rotation

			self.last_twist.linear.x = self.apply_ramp(target_x, self.last_twist.linear.x)
			self.last_twist.linear.y = self.apply_ramp(target_y, self.last_twist.linear.y)
			self.last_twist.angular.z = self.apply_ramp(target_z, self.last_twist.angular.z)

	def publish_loop(self):
		while not rospy.is_shutdown():
			#print(self.last_twist)
			self.pub.publish(self.last_twist)
			self.rate.sleep()

if __name__ == '__main__':
	joy_to_cmd_vel = JoyToCmdVel()
	joy_to_cmd_vel.publish_loop()

