#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class MoveBaseState(EventState):
    '''
    Envoie un objectif de navigation au serveur move_base pour déplacer le robot vers la position donnée.

    Input Keys:
        goal_pose    geometry_msgs/PoseStamped
                     La position et l'orientation cible provenant du DemandeState.

    Output Keys:
        final_pose   geometry_msgs/PoseStamped
                     La dernière position connue du robot.

    Outcomes:
        done     Le robot a atteint la position demandée.
        failed   Un problème est survenu lors de l'envoi ou de l'exécution de l'objectif.

    Pendant l'exécution, la position actuelle du robot est affichée dans la console.
    '''
    def __init__(self):
        super(MoveBaseState, self).__init__(outcomes=['done', 'failed'],
                                              input_keys=['goal_pose'],
                                              output_keys=['final_pose'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._error = False

        # Abonnement pour obtenir la position actuelle du robot (généralement publiée par AMCL dans RViz)
        self._current_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_cb)

    def _pose_cb(self, msg):
        # Conversion du message PoseWithCovarianceStamped en PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._current_pose = pose

    def on_enter(self, userdata):
        goal_pose = userdata.goal_pose
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        
        self._error = False  # Réinitialise l'état d'erreur
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo("Objectif envoyé à move_base : %s" % str(goal_pose))
        except Exception as e:
            Logger.logwarn("Échec de l'envoi de l'objectif move_base:\n%s" % str(e))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'
        
        # Affichage périodique de la position actuelle du robot
        if self._current_pose is not None:
            Logger.loginfo("Position actuelle du robot : %s" % str(self._current_pose.pose))
        else:
            Logger.loginfo("Position actuelle du robot non disponible.")

        # Vérification si le serveur a terminé l'exécution de l'objectif
        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)
            userdata.final_pose = self._current_pose  # On stocke la dernière position connue
            return 'done'
        
        return None

    def on_exit(self, userdata):
        # En cas de sortie prématurée, annulation de l'objectif actif
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Annulation de l'objectif move_base actif.")

