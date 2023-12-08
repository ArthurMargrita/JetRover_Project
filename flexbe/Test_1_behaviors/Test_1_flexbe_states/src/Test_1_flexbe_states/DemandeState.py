#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class DemandeState(EventState):
    '''
    Demande à l'utilisateur de saisir les valeurs du goal_pose (position et orientation).

    Outcomes:
        done: Les valeurs ont été saisies correctement.
        failed: Une erreur est survenue lors de la saisie.

    Output Keys:
        goal_pose    geometry_msgs/PoseStamped
                     Le goal sous forme de PoseStamped à transmettre au state de navigation.
    '''
    def __init__(self):
        super(DemandeState, self).__init__(outcomes=['done', 'failed'],
                                             output_keys=['goal_pose'])
        self._goal_pose = None

    def on_enter(self, userdata):
        try:
            print("\n=== Saisie du goal_pose ===")
            # Saisie de la position
            x = float(input("Entrez la valeur de x : "))
            y = float(input("Entrez la valeur de y : "))
            z = float(input("Entrez la valeur de z : "))
            
            # Saisie de l'orientation en quaternion
            print("Entrez l'orientation en quaternion :")
            qx = float(input("qx : "))
            qy = float(input("qy : "))
            qz = float(input("qz : "))
            qw = float(input("qw : "))
            
            # Création du message PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "map"  # La map prédéfinie de la simulation
            
            pose_stamped.pose.position = Point(x, y, z)
            pose_stamped.pose.orientation = Quaternion(qx, qy, qz, qw)
            
            self._goal_pose = pose_stamped
            Logger.loginfo("Goal pose créé avec succès : %s" % str(pose_stamped))
        except Exception as e:
            Logger.logwarn("Erreur lors de la saisie du goal_pose : %s" % str(e))
            self._goal_pose = None

    def execute(self, userdata):
        if self._goal_pose is not None:
            userdata.goal_pose = self._goal_pose
            return 'done'
        else:
            return 'failed'

    def on_exit(self, userdata):
        # Remise à zéro si nécessaire pour réutiliser cet état ultérieurement.
        self._goal_pose = None

