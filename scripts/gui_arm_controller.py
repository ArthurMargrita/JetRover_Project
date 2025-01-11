#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import actionlib
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class RobotGUI(QWidget):
    def __init__(self):
        super(RobotGUI, self).__init__()

        # Initialiser ROS
        rospy.init_node('robot_gui_arm_controller', anonymous=True)

        # Client Action pour commander les joints du bras
        self.arm_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction
        )
        rospy.loginfo("En attente du serveur Action pour le bras...")
        self.arm_client.wait_for_server()
        rospy.loginfo("Serveur Action pour le bras connecté.")

        # Client Action pour commander le gripper
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction
        )
        rospy.loginfo("En attente du serveur Action pour le gripper...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Serveur Action pour le gripper connecté.")

        # Configuration de la fenêtre principale
        self.setWindowTitle('Robot Controller GUI')
        self.resize(400, 500)

        # Variables pour les positions des joints
        self.joint_positions = [0.0] * 4  # Adaptez ce nombre à vos joints
        self.gripper_position = 0.0

        # Création de la mise en page
        self.layout = QVBoxLayout()

        # Ajout des sliders pour chaque joint du bras
        self.sliders = []
        for i in range(4):  # 4 joints à commander
            label = QLabel('Joint {}'.format(i + 1))
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)  # Limite en centièmes de radians (-3.14 rad)
            slider.setMaximum(314)   # Limite en centièmes de radians (3.14 rad)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, joint=i: self.update_and_send_joint_position(joint, value))

            self.layout.addWidget(label)
            self.layout.addWidget(slider)
            self.sliders.append(slider)

        # Slider pour le gripper
        gripper_label = QLabel('Gripper')
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setMinimum(-50)  # Position minimale du gripper
        self.gripper_slider.setMaximum(120)  # Position maximale
        self.gripper_slider.setValue(0)    # Position initiale
        self.gripper_slider.valueChanged.connect(self.update_and_send_gripper_position)

        self.layout.addWidget(gripper_label)
        self.layout.addWidget(self.gripper_slider)

        # Afficher la mise en page
        self.setLayout(self.layout)

    def update_and_send_joint_position(self, joint, value):
        """Met à jour la position d'un joint spécifique et l'envoie immédiatement."""
        self.joint_positions[joint] = value / 100.0  # Convertir en radians
        rospy.loginfo('Joint {} position mise à jour : {} radians'.format(joint + 1, self.joint_positions[joint]))
        self.send_joint_commands()

    def update_and_send_gripper_position(self, value):
        """Met à jour la position du gripper et l'envoie immédiatement."""
        self.gripper_position = value / 100.0  # Convertir en une valeur entre 0 et 1
        rospy.loginfo('Position gripper mise à jour : {}'.format(self.gripper_position))
        self.send_gripper_command()

    def send_joint_commands(self):
        """Envoie les positions des joints au bras via l'action FollowJointTrajectory."""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']  # Adaptez les noms
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(0.5)  # Temps d'exécution court pour un contrôle fluide

        goal.trajectory.points.append(point)

        rospy.loginfo('Envoi de la trajectoire pour le bras : {}'.format(self.joint_positions))
        self.arm_client.send_goal(goal)

    def send_gripper_command(self):
        """Envoie la position du gripper via l'action FollowJointTrajectory."""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['r_joint']  # Adaptez ce nom si nécessaire
        point = JointTrajectoryPoint()
        point.positions = [self.gripper_position]
        point.time_from_start = rospy.Duration(0.5)

        goal.trajectory.points.append(point)

        rospy.loginfo('Envoi de la trajectoire pour le gripper : {}'.format(self.gripper_position))
        self.gripper_client.send_goal(goal)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotGUI()
    gui.show()
    sys.exit(app.exec_())

