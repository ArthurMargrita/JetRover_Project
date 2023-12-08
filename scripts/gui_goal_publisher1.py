#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_srvs.srv import Empty

import math
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QLineEdit,
    QLabel,
    QPushButton,
    QWidget,
    QHBoxLayout,
    QFormLayout,
)

def euler_to_quaternion(yaw):
    """
    Convertit une rotation autour de l'axe Z (yaw) en quaternion.
    """
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class GoalPublisherApp(QMainWindow):
    def __init__(self):
        super(GoalPublisherApp, self).__init__()

        # Initialisation du publisher ROS
        rospy.init_node("goal_publisher_gui", anonymous=True)
        self.publisher = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

        # Initialisation du subscriber pour le topic /amcl_pose
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        # Configuration de l'interface utilisateur
        self.setWindowTitle("Goal Publisher")
        self.setGeometry(100, 100, 400, 300)

        # Layout principal
        main_layout = QVBoxLayout()

        # Champs pour la position et la rotation
        form_layout = QFormLayout()

        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.yaw_input = QLineEdit()

        form_layout.addRow(QLabel("Position X :"), self.x_input)
        form_layout.addRow(QLabel("Position Y :"), self.y_input)
        form_layout.addRow(QLabel("Rotation (Yaw, en radians) :"), self.yaw_input)

        main_layout.addLayout(form_layout)

        # Boutons
        button_layout = QHBoxLayout()

        self.publish_button = QPushButton("Publier")
        self.publish_button.clicked.connect(self.publish_goal)

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop)

        self.reset_button = QPushButton("Réinitialiser AMCL")  # Bouton pour reset AMCL
        self.reset_button.clicked.connect(self.reset_amcl)

        button_layout.addWidget(self.publish_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.reset_button)

        main_layout.addLayout(button_layout)

        # Configuration du widget principal
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Variables pour stocker les données
        self.goal_position = None
        self.robot_position = None

        # Démarrer un timer pour la vérification périodique
        rospy.Timer(rospy.Duration(1), self.check_positions)

    def publish_goal(self):
        """
        Publie un message MoveBaseActionGoal sur le topic /move_base/goal.
        """
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = 0
            yaw = float(self.yaw_input.text())

            goal_msg = MoveBaseActionGoal()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.goal_id.stamp = rospy.Time.now()
            goal_msg.goal_id.id = "goal_gui"

            goal_msg.goal.target_pose.header.frame_id = "map"
            goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
            goal_msg.goal.target_pose.pose.position.x = x
            goal_msg.goal.target_pose.pose.position.y = y
            goal_msg.goal.target_pose.pose.position.z = z
            goal_msg.goal.target_pose.pose.orientation = euler_to_quaternion(yaw)

            rospy.loginfo("Envoi du goal : %s", goal_msg)
            self.publisher.publish(goal_msg)

            self.goal_position = (x, y)

        except ValueError:
            rospy.logwarn("Veuillez entrer des valeurs numériques valides.")

    def pose_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def check_positions(self, event):
        if self.goal_position and self.robot_position:
            goal_x, goal_y = self.goal_position
            robot_x, robot_y = self.robot_position
            distance = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
            rospy.loginfo("Distance entre le robot et le goal : %.2f", distance)
            if distance <= 0.2:
                rospy.loginfo("Le robot a atteint la position cible !")
                self.goal_position = None

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
        Réinitialise la position d'AMCL en appelant le service /global_localization.
        """
        rospy.loginfo("Réinitialisation d'AMCL en cours...")
        rospy.wait_for_service("/global_localization")
        try:
            reset_service = rospy.ServiceProxy("/global_localization", Empty)
            reset_service()
            rospy.loginfo("AMCL réinitialisé avec succès.")
        except rospy.ServiceException as e:
            rospy.logerr("Erreur lors de l'appel au service: %s", e)

if __name__ == "__main__":
    try:
        app = QApplication(sys.argv)
        window = GoalPublisherApp()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        QApplication.instance().quit()

