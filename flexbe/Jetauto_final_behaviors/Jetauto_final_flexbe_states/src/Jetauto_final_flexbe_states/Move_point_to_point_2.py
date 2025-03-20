#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import math
from geometry_msgs.msg import Quaternion


class Move_point_to_point_2(EventState):
    '''
    Envoie un objectif de navigation au serveur move_base pour déplacer le robot vers la position donnée.

    Input Keys:
        x, y, angle  (float, float, float)
                    Coordonnées cibles et orientation en degrés.

    Outcomes:
        done     Le robot a atteint la position demandée.
        failed   Un problème est survenu lors de l'envoi ou de l'exécution de l'objectif.
    '''
    def __init__(self):
        super(Move_point_to_point_2, self).__init__(outcomes=['done', 'failed'],
                                                    input_keys=['x', 'y', 'angle'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._error = False

    def euler_to_quaternion(self, yaw_degrees):
        """
        Convertit une rotation autour de l'axe Z (yaw) en quaternion.
        """
        yaw = math.radians(yaw_degrees)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def _convert_to_pose_stamped(self, x, y, angle):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        quaternion = self.euler_to_quaternion(angle)
        pose.pose.orientation = quaternion
        
        return pose

    def on_enter(self, userdata):
        goal_pose = self._convert_to_pose_stamped(userdata.x, userdata.y, userdata.angle)
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        
        self._error = False  # Réinitialisation de l'erreur
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo("Objectif envoyé à move_base : %s" % str(goal_pose))
        except Exception as e:
            Logger.logwarn("Échec de l'envoi de l'objectif move_base:\n%s" % str(e))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'

        # Vérification de la fin de l'objectif
        if self._client.has_result(self._action_topic):
            Logger.loginfo("Le robot est arrivé à la destination.")
            return 'done'
        
        return None

    def on_exit(self, userdata):
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Annulation de l'objectif move_base actif.")
