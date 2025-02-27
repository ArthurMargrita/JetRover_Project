#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy

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


class JoyAxisMonitor:
    def __init__(self):
        rospy.init_node('joy_axis_monitor', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # Stocke les dernières valeurs des axes et boutons
        self.last_axes = None
        self.last_buttons = None
        
        rospy.spin()

    def joy_callback(self, data):
        """Affiche les axes qui changent significativement et les boutons pressés."""
        # Initialisation au premier message reçu
        if self.last_axes is None:
            self.last_axes = list(data.axes)
        if self.last_buttons is None:
            self.last_buttons = list(data.buttons)

        # Vérification des axes (évite les petites variations)
        for i in range(len(data.axes)):
            if abs(data.axes[i] - self.last_axes[i]) > 0.01:
                rospy.loginfo("Axe {} changé : {:.3f}".format(i, data.axes[i]))
                self.last_axes[i] = data.axes[i]

        # Vérification des boutons (affiche uniquement lorsqu'ils sont pressés)
        for i in range(len(data.buttons)):
            if data.buttons[i] == 1 and self.last_buttons[i] == 0:
                rospy.loginfo("Bouton {} pressé".format(i))

            # Mise à jour des valeurs précédentes
            self.last_buttons[i] = data.buttons[i]

if __name__ == '__main__':
    JoyAxisMonitor()

