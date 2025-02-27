#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


"""

AXES & BUTTON

axes[0] : left joy gauche/droite
axes[1] : left joy haut/bas
axes[2] : right joy haut/bas
axes[3] : right joy gauche/droite
axes[4] : flèche gauche /droite
axes[5] : flèches haut/bas

button[0] : 1 ou triangle
button[1] : 2 ou rond
button[2] : 3 ou x
button[3] : 4 ou carré
button[4] : L1
button[5] : R1
button[6] : L2
button[7] : R2
button[8] : select
button[9] : start
button[10] : left joy pressed
button[10] : right joy pressed

"""

class JoyToCmdVel:
		def __init__(self):
				rospy.init_node('joy_to_cmd_vel', anonymous=True)
				self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
				rospy.Subscriber('/joy', Joy, self.joy_callback)

				# Mode de contrôle (True = déplacement, False = bras)
				self.mode = 0

				self.last_twist = Twist()
				self.target_twist = Twist()
		
				self.last_axes = [0] * 6 
		
				self.rate = rospy.Rate(30)  # fréquence de publication des messages twist

				self.max_accel = 0.05  # Ajuste cette valeur pour un démarrage plus ou moins rapide pour la rampe

		def joy_callback(self, data):
				"""Callback pour traiter les données du joystick."""
		
				if data.buttons[8]:  # Bouton Select pour changer de mode
						if self.mode == 0 :
								self.mode = 1
								rospy.loginfo("Mode changé : {}".format("Déplacement 1" ))
						if self.mode == 1:
								self.mode = 2
								rospy.loginfo("Mode changé : {}".format("Bras" ))
						if self.mode == 2:
								self.mode = 0
								rospy.loginfo("Mode changé : {}".format("Déplacement 0" ))

				if self.mode == 0 or self.mode == 1:
						if data.buttons[10]:  # Arrêt immédiat si le bouton 10 est pressé
								rospy.loginfo("Arrêt d'urgence activé !")
								self.target_twist = Twist()  
								self.last_axes = [0] * len(self.last_axes)
								return 

						# Mise à jour des axes
						self.last_axes = data.axes

						if self.mode == 0:
								self.target_twist.linear.x = self.last_axes[1] / 2.5  
			
						self.target_twist.angular.z = self.last_axes[0] / 2 
						self.target_twist.linear.y = self.last_axes[3] / 5  
				
						if self.mode == 1:
								if data.buttons[5]:
										self.target_twist.linear.x = 0.1
								if data.buttons[4]:
										self.target_twist.linear.x = -0.1
								if data.buttons[6]:
										self.target_twist.linear.x = -0.3
								if data.buttons[7]:
										self.target_twist.linear.x = 0.3
						
				if self.mode == 1:
						if data.buttons[10]:  # Arrêt immédiat si le bouton 10 est pressé
								rospy.loginfo("Arrêt d'urgence activé !")
								self.target_twist = Twist()  
								self.last_axes = [0] * len(self.last_axes)
								return 

						# Mise à jour des axes
						self.last_axes = data.axes

						if self.mode:  # Mode déplacement
								self.target_twist.linear.x = self.last_axes[1] / 2.5  
								self.target_twist.angular.z = self.last_axes[0] / 2 
								self.target_twist.linear.y = self.last_axes[3] / 5

		def apply_ramp(self, current, target, max_step):
				"""Applique une rampe d'accélération pour lisser les variations de vitesse."""
				delta = target - current
				if abs(delta) > max_step:
						delta = max_step if delta > 0 else -max_step
				return current + delta

		def publish_loop(self):
				"""Publie les commandes en continu à une fréquence fixe."""
				while not rospy.is_shutdown():
			
						if self.mode == 0 or self.mode == 1:
								# Appliquer la rampe d'accélération sur chaque composante
								self.last_twist.linear.x = self.apply_ramp(self.last_twist.linear.x, 
																		   self.target_twist.linear.x, 
																		   self.max_accel)
				
								self.last_twist.linear.y = self.apply_ramp(self.last_twist.linear.y, 
																		   self.target_twist.linear.y, 
																		   self.max_accel)
				
								self.last_twist.angular.z = self.apply_ramp(self.last_twist.angular.z, 
																			self.target_twist.angular.z, 
																			self.max_accel)

								# Publier les commandes
								self.pub.publish(self.last_twist)
								rospy.loginfo(self.last_twist)
								self.rate.sleep()

if __name__ == '__main__':
		joy_to_cmd_vel = JoyToCmdVel()
		joy_to_cmd_vel.publish_loop()
