#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import requests
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveTableState(EventState):
    '''
    Envoie l'objectif de navigation fourni via l'input (goal_pose_key) au serveur move_base.

    Le robot se deplace vers la position donnee. Une fois l'objectif atteint, le state 
    effectue une requete POST vers l'endpoint /remove_first pour supprimer le premier 
    element de la liste du serveur, puis renvoie l'outcome 'Table'.

    Input Keys:
        goal_pose_key    geometry_msgs/PoseStamped
                         L'objectif de navigation envoye par OrderReceptionState_2.

    Outcomes:
        Table      Le robot a atteint la position de la table.
        failed     Une erreur s'est produite lors de l'envoi ou de l'execution de l'objectif.
    '''
    
    def __init__(self):
        super(MoveTableState, self).__init__(outcomes=['Table', 'failed'],
                                             input_keys=['goal_pose_key'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._error = False
        self.server_url = 'http://192.168.149.71:5000'  # A adapter si besoin

    def on_enter(self, userdata):
        # Envoi de l'objectif de navigation recu en input.
        goal = MoveBaseGoal()
        goal.target_pose = userdata.goal_pose_key
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo("Table goal sent: %s" % str(userdata.goal_pose_key))
        except Exception as e:
            Logger.logwarn("Failed to send Table goal: %s" % str(e))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'
        # Lorsque l'objectif est atteint, supprimer le premier element du serveur et renvoyer l'outcome.
        if self._client.has_result(self._action_topic):
            Logger.loginfo("Table goal reached.")
            try:
                requests.post(self.server_url + '/remove_first', timeout=1.0)
                Logger.loginfo("First term removed from server list.")
            except Exception as e:
                Logger.logwarn("Failed to remove first command: %s" % str(e))
            return 'Table'
        return None

    def on_exit(self, userdata):
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Table goal cancelled.")

