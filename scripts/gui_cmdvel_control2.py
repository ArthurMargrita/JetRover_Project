#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from PyQt4.QtGui import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QLabel
from PyQt4.QtCore import Qt

class RobotController(QWidget):
    def __init__(self):
        super(RobotController, self).__init__()
        self.init_ros()
        self.init_ui()

        # Vitesse initiale
        self.speed_x = 0.3
        self.speed_y = 0.2
        self.speed_ang = 0.5

    def init_ros(self):
        """Initialise le nœud ROS et le publisher."""
        rospy.init_node("robot_controller_direction_gui", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def send_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publie un message Twist sur le topic /cmd_vel."""
        twist = Twist()
        twist.linear.x = linear_x * self.speed_x
        twist.linear.y = linear_y * self.speed_y
        twist.angular.z = angular_z * self.speed_ang
        self.pub.publish(twist)

    def update_speed_x(self, value):
        """Met à jour la vitesse selon la position du slider."""
        self.speed_x = value / 100.0  # Convertir la valeur du slider (0-100) en 0.0-1.0
        self.speed_x_label.setText("Vitesse X: {:.2f}".format(self.speed_x))
        
    def update_speed_y(self, value):
        """Met à jour la vitesse selon la position du slider."""
        self.speed_y = value / 100.0  # Convertir la valeur du slider (0-100) en 0.0-1.0
        self.speed_y_label.setText("Vitesse Y: {:.2f}".format(self.speed_y))
        
    def update_speed_ang(self, value):
        """Met à jour la vitesse selon la position du slider."""
        self.speed_ang = value / 100.0  # Convertir la valeur du slider (0-100) en 0.0-1.0
        self.speed_ang_label.setText("Vitesse ang: {:.2f}".format(self.speed_ang))
        	


    def init_ui(self):
        """Initialise l'interface graphique."""
        self.setWindowTitle("Control Jetauto Pro")
        self.setGeometry(100, 100, 400, 400)

        # Layouts
        main_layout = QVBoxLayout()
        movement_layout = QHBoxLayout()
        angular_layout = QHBoxLayout()
        slider_layout = QVBoxLayout()

        # Boutons de contrôle linéaire
        btn_forward = QPushButton("forward")
        btn_backward = QPushButton("backward")
        btn_left = QPushButton("left")
        btn_right = QPushButton("right")
        btn_stop = QPushButton("STOP")

        # Boutons de contrôle angulaire
        btn_rotate_left = QPushButton("turn left")
        btn_rotate_right = QPushButton("turn right")

        # Slider pour la vitesse
        self.speed_x_slider = QSlider(Qt.Horizontal)
        self.speed_x_slider.setMinimum(0)
        self.speed_x_slider.setMaximum(100)
        self.speed_x_slider.setValue(30)  # Valeur initiale (0.5)
        self.speed_x_slider.setTickInterval(10)
        self.speed_x_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_x_slider.valueChanged.connect(self.update_speed_x)
        
        self.speed_y_slider = QSlider(Qt.Horizontal)
        self.speed_y_slider.setMinimum(0)
        self.speed_y_slider.setMaximum(100)
        self.speed_y_slider.setValue(20)  # Valeur initiale (0.5)
        self.speed_y_slider.setTickInterval(10)
        self.speed_y_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_y_slider.valueChanged.connect(self.update_speed_y)
        
        self.speed_ang_slider = QSlider(Qt.Horizontal)
        self.speed_ang_slider.setMinimum(0)
        self.speed_ang_slider.setMaximum(100)
        self.speed_ang_slider.setValue(50)  # Valeur initiale (0.5)
        self.speed_ang_slider.setTickInterval(10)
        self.speed_ang_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_ang_slider.valueChanged.connect(self.update_speed_ang)

        # Étiquette pour afficher la vitesse actuelle
        self.speed_x_label = QLabel("Vitesse X: 0.30")
        self.speed_x_label.setAlignment(Qt.AlignCenter)
        
        self.speed_y_label = QLabel("Vitesse Y: 0.20")
        self.speed_y_label.setAlignment(Qt.AlignCenter)
        
        self.speed_ang_label = QLabel("Vitesse ang: 0.50")
        self.speed_ang_label.setAlignment(Qt.AlignCenter)

        # Connexion des boutons linéaires aux fonctions
        btn_forward.clicked.connect(lambda: self.send_velocity(1.0, 0.0, 0.0))
        btn_backward.clicked.connect(lambda: self.send_velocity(-1.0, 0.0, 0.0))
        btn_left.clicked.connect(lambda: self.send_velocity(0.0, 1.0, 0.0))
        btn_right.clicked.connect(lambda: self.send_velocity(0.0, -1.0, 0.0))
        btn_stop.clicked.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))

        # Connexion des boutons angulaires aux fonctions
        btn_rotate_left.clicked.connect(lambda: self.send_velocity(0.0, 0.0, 1.0))
        btn_rotate_right.clicked.connect(lambda: self.send_velocity(0.0, 0.0, -1.0))

        # Ajout des boutons aux layouts
        main_layout.addWidget(btn_forward)
        movement_layout.addWidget(btn_left)
        movement_layout.addWidget(btn_stop)
        movement_layout.addWidget(btn_right)
        main_layout.addLayout(movement_layout)
        main_layout.addWidget(btn_backward)

        angular_layout.addWidget(btn_rotate_left)
        angular_layout.addWidget(btn_rotate_right)
        main_layout.addLayout(angular_layout)

        # Ajout du slider et de l'étiquette
        slider_layout.addWidget(self.speed_x_label)
        slider_layout.addWidget(self.speed_x_slider)
        slider_layout.addWidget(self.speed_y_label)
        slider_layout.addWidget(self.speed_y_slider)
        slider_layout.addWidget(self.speed_ang_label)
        slider_layout.addWidget(self.speed_ang_slider)
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
    finally:
        mecanum_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        mecanum_pub.publish(twist)
		

