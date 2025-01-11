#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from PyQt4.QtGui import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QLabel
from PyQt4.QtCore import Qt, QTimer

class RobotController(QWidget):
    def __init__(self):
        super(RobotController, self).__init__()
        self.init_ros()
        self.init_ui()

        self.speed = 0.5  # Vitesse par défaut
        self.current_speed = 0.0  # Vitesse actuelle (pour les transitions douces)
        self.acceleration_step = 0.05  # Pas d'accélération
        self.update_rate = 50  # Intervalle du timer (en ms)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_velocity)

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

    def init_ros(self):
        """Initialise le nœud ROS et le publisher."""
        rospy.init_node("robot_controller_direction_keyboard", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def send_velocity(self):
        """Publie un message Twist sur le topic /cmd_vel."""
        twist = Twist()
        twist.linear.x = self.current_speed * self.target_linear_x
        twist.linear.y = self.current_speed * self.target_linear_y
        twist.angular.z = self.current_speed * self.target_angular_z
        self.pub.publish(twist)

    def update_speed(self, value):
        """Met à jour la vitesse selon la position du slider."""
        self.speed = value / 100.0  # Convertir la valeur du slider (0-100) en 0.0-1.0
        self.speed_label.setText("Vitesse : {:.2f}".format(self.speed))

    def update_velocity(self):
        """Met à jour progressivement la vitesse actuelle jusqu'à la vitesse cible."""
        if self.current_speed < self.speed:
            self.current_speed = min(self.current_speed + self.acceleration_step, self.speed)
        elif self.current_speed > self.speed:
            self.current_speed = max(self.current_speed - self.acceleration_step, 0.0)

        self.send_velocity()

    def start_movement(self, linear_x, linear_y, angular_z):
        """Démarre un mouvement progressif vers une direction cible."""
        self.target_linear_x = linear_x
        self.target_linear_y = linear_y
        self.target_angular_z = angular_z
        self.timer.start(self.update_rate)

    def stop_movement(self):
        """Arrête le mouvement."""
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.timer.stop()
        self.current_speed = 0.0
        self.send_velocity()

    def init_ui(self):
        """Initialise l'interface graphique."""
        self.setWindowTitle("Contrôleur de Robot")
        self.setGeometry(100, 100, 400, 400)

        # Layouts
        main_layout = QVBoxLayout()
        movement_layout = QHBoxLayout()
        angular_layout = QHBoxLayout()
        slider_layout = QVBoxLayout()

        # Boutons de contrôle linéaire
        btn_forward = QPushButton("Avancer")
        btn_backward = QPushButton("Reculer")
        btn_left = QPushButton("Gauche")
        btn_right = QPushButton("Droite")
        btn_stop = QPushButton("Arrêter")

        # Slider pour la vitesse
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(50)  # Valeur initiale (0.5)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.valueChanged.connect(self.update_speed)

        # Étiquette pour afficher la vitesse actuelle
        self.speed_label = QLabel("Vitesse : 0.50")
        self.speed_label.setAlignment(Qt.AlignCenter)

        # Connexion des boutons linéaires aux fonctions
        btn_forward.pressed.connect(lambda: self.start_movement(1.0, 0.0, 0.0))
        btn_backward.pressed.connect(lambda: self.start_movement(-1.0, 0.0, 0.0))
        btn_left.pressed.connect(lambda: self.start_movement(0.0, 1.0, 0.0))
        btn_right.pressed.connect(lambda: self.start_movement(0.0, -1.0, 0.0))
        btn_stop.clicked.connect(self.stop_movement)

        # Ajout des boutons aux layouts
        main_layout.addWidget(btn_forward)
        movement_layout.addWidget(btn_left)
        movement_layout.addWidget(btn_stop)
        movement_layout.addWidget(btn_right)
        main_layout.addLayout(movement_layout)
        main_layout.addWidget(btn_backward)

        # Ajout du slider et de l'étiquette
        slider_layout.addWidget(self.speed_label)
        slider_layout.addWidget(self.speed_slider)
        main_layout.addLayout(slider_layout)

        # Appliquer le layout principal
        self.setLayout(main_layout)

if __name__ == "__main__":
    import sys
    try:
        app = QApplication(sys.argv)
        controller = RobotController()
        controller.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
		mecanum_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        mecanum_pub.publish(twist)
        pass
