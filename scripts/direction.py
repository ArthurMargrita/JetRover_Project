#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from PyQt4.QtGui import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PyQt4.QtCore import Qt
from geometry_msgs.msg import Twist

class RobotOmniGUI(QWidget):
    def __init__(self):
        super(RobotOmniGUI, self).__init__()

        # Initialiser ROS
        rospy.init_node('robot_omni_gui_controller', anonymous=True)

        # Publisher pour le topic cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/ubuntu/hiwonder_controller/cmd_vel', Twist, queue_size=10)

        # Configuration de la fenêtre principale
        self.setWindowTitle('Robot Omni Controller GUI')
        self.resize(400, 300)

        # Création de la mise en page
        self.layout = QVBoxLayout()

        # Labels et sliders pour les axes x, y et rotation
        self.x_label = QLabel('Vitesse lineaire X')
        self.x_slider = QSlider(Qt.Horizontal)
        self.x_slider.setMinimum(-100)  # Valeur minimale
        self.x_slider.setMaximum(100)  # Valeur maximale
        self.x_slider.setValue(0)      # Position initiale
        self.x_slider.valueChanged.connect(self.update_twist)

        self.y_label = QLabel('Vitesse lineaire Y')
        self.y_slider = QSlider(Qt.Horizontal)
        self.y_slider.setMinimum(-100)
        self.y_slider.setMaximum(100)
        self.y_slider.setValue(0)
        self.y_slider.valueChanged.connect(self.update_twist)

        self.angular_label = QLabel('Vitesse angulaire Z')
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setMinimum(-100)
        self.angular_slider.setMaximum(100)
        self.angular_slider.setValue(0)
        self.angular_slider.valueChanged.connect(self.update_twist)

        # Bouton pour arrêter le robot
        self.stop_button = QPushButton('Arreter')
        self.stop_button.clicked.connect(self.stop_robot)

        # Ajouter les widgets à la mise en page
        self.layout.addWidget(self.x_label)
        self.layout.addWidget(self.x_slider)
        self.layout.addWidget(self.y_label)
        self.layout.addWidget(self.y_slider)
        self.layout.addWidget(self.angular_label)
        self.layout.addWidget(self.angular_slider)
        self.layout.addWidget(self.stop_button)

        # Afficher la mise en page
        self.setLayout(self.layout)

        # Objet Twist pour stocker les commandes
        self.twist = Twist()

    def update_twist(self):
        """Met à jour le message Twist en fonction des sliders et publie sur /cmd_vel."""
        self.twist.linear.x = self.x_slider.value() / 100.0  # Conversion en m/s
        self.twist.linear.y = self.y_slider.value() / 100.0
        self.twist.angular.z = self.angular_slider.value() / 100.0
        rospy.loginfo("Vitesse mise à jour : X={}, Y={}, Z={}".format(self.twist.linear.x, self.twist.linear.y, self.twist.angular.z))

        self.cmd_vel_pub.publish(self.twist)

    def stop_robot(self):
        """Arrête le robot en remettant toutes les vitesses à zéro."""
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.angular.z = 0.0
        rospy.loginfo("Arrêt du robot.")
        self.cmd_vel_pub.publish(self.twist)
        self.x_slider.setValue(0)
        self.y_slider.setValue(0)
        self.angular_slider.setValue(0)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotOmniGUI()
    gui.show()
    sys.exit(app.exec_())

