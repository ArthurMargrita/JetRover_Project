#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_srvs.srv import Empty
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QLineEdit, QLabel,
    QPushButton, QWidget, QHBoxLayout, QFormLayout
)
import math

def quaternion_to_euler(q):
    """
    Convertit un quaternion en un angle de rotation autour de l'axe Z (yaw en degrés).
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)

def euler_to_quaternion(yaw):
    """
    Convertit un angle de rotation autour de l'axe Z (yaw en degrés) en quaternion.
    """
    yaw = math.radians(yaw)  # Convertir en radians
    q = Quaternion()
    q.w = math.cos(yaw / 2)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2)
    return q

class GoalPublisherApp(QMainWindow):
    def __init__(self):
        super(GoalPublisherApp, self).__init__()

        # Initialisation du publisher ROS
        rospy.init_node("goal_publisher_gui", anonymous=True)
        self.publisher = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1, latch=True)
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        # Configuration de l'interface utilisateur
        self.setWindowTitle("Goal Publisher")
        self.setGeometry(100, 100, 400, 350)

        # Layout principal
        main_layout = QVBoxLayout()
        form_layout = QFormLayout()

        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.yaw_input = QLineEdit()

        form_layout.addRow(QLabel("Position X :"), self.x_input)
        form_layout.addRow(QLabel("Position Y :"), self.y_input)
        form_layout.addRow(QLabel("Rotation (Yaw, en degré) :"), self.yaw_input)

        main_layout.addLayout(form_layout)

        # Label pour afficher la position actuelle du robot
        self.robot_position_label = QLabel("Position actuelle : Inconnue")
        main_layout.addWidget(self.robot_position_label)

        # Boutons
        button_layout = QHBoxLayout()

        self.publish_button = QPushButton("Publier")
        self.publish_button.clicked.connect(self.publish_goal)

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop)

        self.reset_button = QPushButton("Réinitialiser AMCL")
        self.reset_button.clicked.connect(self.reset_amcl)

        self.check_position_button = QPushButton("Check Position")
        self.check_position_button.clicked.connect(self.check_position)

        button_layout.addWidget(self.publish_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.reset_button)
        button_layout.addWidget(self.check_position_button)

        main_layout.addLayout(button_layout)

        # Configuration du widget principal
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Variables pour stocker les données
        self.goal_position = None
        self.robot_position = None
        self.robot_yaw = None

    def publish_goal(self):
        """
        Publie un message MoveBaseActionGoal sur le topic /move_base/goal.
        """
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            yaw = float(self.yaw_input.text())

            # Création du message MoveBaseActionGoal
            goal_msg = MoveBaseActionGoal()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = rospy.Time.now()

            # Définition de la cible
            goal_msg.goal.target_pose.header.frame_id = "map"
            goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
            goal_msg.goal.target_pose.pose.position.x = x
            goal_msg.goal.target_pose.pose.position.y = y
            goal_msg.goal.target_pose.pose.orientation = euler_to_quaternion(yaw)

            rospy.loginfo("Envoi du goal : %s", goal_msg)
            self.publisher.publish(goal_msg)

            self.goal_position = (x, y)

        except ValueError:
            rospy.logwarn("Veuillez entrer des valeurs numériques valides.")

    def pose_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_yaw = quaternion_to_euler(msg.pose.pose.orientation)

    def stop(self):
        rospy.loginfo("Annulation du goal en cours...")
        cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        rospy.sleep(1)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)
        rospy.loginfo("Goal annulé.")
        self.goal_position = None

    def reset_amcl(self):
        """
        Réinitialise la position d'AMCL en appelant le service /global_localization
        puis fixe la position initiale à (0,0,0).
        """
        rospy.loginfo("Réinitialisation d'AMCL en cours...")
        rospy.wait_for_service("/global_localization")
    
        try:
            reset_service = rospy.ServiceProxy("/global_localization", Empty)
            reset_service()
            rospy.loginfo("AMCL réinitialisé avec succès.")

            rospy.sleep(1)

            pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
            rospy.sleep(1)

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.pose.position.x = 0.0
            pose_msg.pose.pose.position.y = 0.0
            pose_msg.pose.pose.orientation.w = 1.0

            for _ in range(5):
                pub.publish(pose_msg)
                rospy.sleep(0.2)

            rospy.loginfo("Position initiale définie à (0,0).")
            self.robot_position_label.setText("Position actuelle : X=0.00, Y=0.00")

        except rospy.ServiceException as e:
            rospy.logerr("Erreur lors de l'appel au service: %s", e)

    def check_position(self):
        """
        Affiche la position actuelle du robot avec AMCL.
        """
        if self.robot_position:
            self.robot_position_label.setText("Position actuelle : X={:.2f}, Y={:.2f}, Yaw (deg)={:.2f}".format(
                self.robot_position[0], self.robot_position[1], self.robot_yaw))
        else:
            self.robot_position_label.setText("Position actuelle : Inconnue")

if __name__ == "__main__":
    try:
        app = QApplication(sys.argv)
        window = GoalPublisherApp()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        QApplication.instance().quit()

