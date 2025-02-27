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
        self.mode = True

        # Dernière commande valide
        self.last_twist = Twist()

        # Dernières valeurs d’axes mémorisées (évite les retours à zéro intempestifs)
        self.last_axes = [0] * 6  # On suppose que 6 axes sont utilisés

        # Fréquence de publication plus élevée
        self.rate = rospy.Rate(30)  # 30 Hz pour une meilleure fluidité

    def joy_callback(self, data):
        """Callback pour traiter les données du joystick."""
        
        # Vérification et mise à jour du mode (bouton Select)
        if data.buttons[8]:  # Bouton Select pour changer de mode
            self.mode = not self.mode
            rospy.loginfo("Mode changé : {}".format("Déplacement" if self.mode else "Bras"))

        # Arrêt immédiat si le bouton 10 est pressé
        if data.buttons[10]:  
            rospy.loginfo("Arrêt d'urgence activé !")
            self.last_twist = Twist()  # Remet toutes les valeurs à zéro
            self.last_axes = [0] * len(self.last_axes)  # Réinitialise les axes
            return  # On sort immédiatement du callback

        self.last_axes = data.axes

        self.last_twist.linear.x = self.last_axes[1]/3  # Avant/arrière (joystick gauche)

        self.last_twist.angular.z = self.last_axes[0]/3  # Rotation

        self.last_twist.linear.y = self.last_axes[3]/5  # Translation latérale (roues omni)


    def publish_loop(self):
        """Publie les commandes en continu à une fréquence fixe."""
        while not rospy.is_shutdown():
            self.pub.publish(self.last_twist)
            rospy.loginfo(self.last_twist)
            self.rate.sleep()

if __name__ == '__main__':
    joy_to_cmd_vel = JoyToCmdVel()
    joy_to_cmd_vel.publish_loop()

