# -*- coding: utf-8 -*-
#!/usr/bin/env python
 
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
 
def send_trajectory():
    # Initialisation du nœud ROS
    rospy.init_node('send_trajectory', anonymous=True)
 
    # Publisher pour envoyer les trajectoires au contrôleur
    pub = rospy.Publisher('/ubuntu/arm_controller/command', JointTrajectory, queue_size=10)
 
    # Attendre que le topic soit prêt
    rospy.sleep(2)
 
    # Créer un message JointTrajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']  # Remplacez par les noms réels des joints
    point = JointTrajectoryPoint()
 
    # Définir les positions cibles pour les joints
    point.positions = [0.5, -0.3, 1.2, -0.8, 0.0]  # Positions des joints en radians
    point.time_from_start = rospy.Duration(3)  # Temps pour atteindre les positions
 
    # Ajouter le point à la trajectoire
    trajectory.points.append(point)
 
    # Envoyer la commande
    rospy.loginfo("Envoi de la trajectoire : %s", point.positions)
    pub.publish(trajectory)
 
    # Attendre pour s'assurer que la commande est envoyée
    rospy.sleep(1)
 
 
if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass
