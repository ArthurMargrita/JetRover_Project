#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import actionlib
from PyQt4.QtGui import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PyQt4.QtCore import Qt
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class RobotGUI(QWidget):
    def __init__(self):
        super(RobotGUI, self).__init__()

        # Initialiser ROS
        rospy.init_node('robot_gui_controller', anonymous=True)

        # Client Action pour commander les joints
        self.client = actionlib.SimpleActionClient(
            '/ubuntu/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction
        )

        rospy.loginfo("En attente du serveur Action...")
        self.client.wait_for_server()
        rospy.loginfo("Serveur Action connecté.")

        # Configuration de la fenêtre principale
        self.setWindowTitle('Robot Controller GUI')
        self.resize(400, 300)

        # Variables pour les positions des joints
        self.joint_positions = [0.0] * 5  # Ajustez selon le nombre de joints

        # Création de la mise en page
        self.layout = QVBoxLayout()

        # Ajout des sliders pour chaque joint
        self.sliders = []
        for i in range(5):  # Ajustez le nombre de sliders pour vos joints
            label = QLabel('Joint {}'.format(i + 1))
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)  # Limite en degrés (-180° -> -3.14 radians)
            slider.setMaximum(314)   # Limite en degrés (180° -> 3.14 radians)
            slider.setValue(0)       # Position initiale à 0 radians
            slider.valueChanged.connect(lambda value, joint=i: self.update_joint_position(joint, value))

            self.layout.addWidget(label)
            self.layout.addWidget(slider)
            self.sliders.append(slider)

        # Bouton pour envoyer les commandes
        self.send_button = QPushButton('Envoyer Commandes')
        self.send_button.clicked.connect(self.send_joint_commands)
        self.layout.addWidget(self.send_button)

        # Afficher la mise en page
        self.setLayout(self.layout)

    def update_joint_position(self, joint, value):
        """Met à jour la position d'un joint spécifique."""
        self.joint_positions[joint] = value / 100.0  # Convertir en radians
        rospy.loginfo('Joint {} position mise à jour : {} radians'.format(joint + 1, self.joint_positions[joint]))

    def send_joint_commands(self):
        """Envoie les positions des joints au robot via l'action FollowJointTrajectory."""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']  # Ajustez selon vos noms de joints
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(2)  # Temps pour atteindre les positions

        goal.trajectory.points.append(point)

        rospy.loginfo('Envoi de la trajectoire...')
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo('Trajectoire exécutée.')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotGUI()
    gui.show()
    sys.exit(app.exec_())
