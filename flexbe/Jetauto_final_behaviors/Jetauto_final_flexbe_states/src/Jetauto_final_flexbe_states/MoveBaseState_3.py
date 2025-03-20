#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import requests
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveBaseState_3(EventState):
    '''
    Envoie deux objectifs de navigation consécutifs au serveur move_base.
    
    Dans un premier temps, le robot se déplace vers une position fixe (-1.5, -2.5, 0) avec une orientation de 0°.
    Il attend ensuite 3 secondes.
    Puis, il envoie le second objectif reçu via l'input key "goal_pose_key" (provenant du state OrderReceptionState_2)
    et attend d'atteindre ce point.
    Après une nouvelle attente de 3 secondes, il efface le premier terme de la liste du serveur (via l'endpoint /remove_first)
    et renvoie l'outcome 'done'.
    
    Input Keys:
        goal_pose_key    geometry_msgs/PoseStamped
                         L'objectif de navigation envoyé par le state OrderReceptionState_2.
    
    Outcomes:
        done     La séquence complète est réalisée.
        failed   Un problème est survenu lors de l'envoi ou l'exécution d'un objectif.
    '''
    
    def __init__(self):
        super(MoveBaseState_3, self).__init__(outcomes=['done', 'failed'],
                                              input_keys=['goal_pose_key'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self.server_url = 'http://192.168.149.71:5000'  # À adapter si besoin
        
        self._step = None
        self._timer_start = None
        self._error = False

    def on_enter(self, userdata):
        self._step = 0
        self._error = False
        
        # Envoi du premier objectif fixe : position (-1.5, -2.5, 0) et orientation 0° (Quaternion: 0,0,0,1)
        first_goal = MoveBaseGoal()
        first_pose = PoseStamped()
        first_pose.header.stamp = rospy.Time.now()
        first_pose.header.frame_id = "map"
        first_pose.pose.position.x = 3.0
        first_pose.pose.position.y = -1.0
        first_pose.pose.position.z = 0.0
        first_pose.pose.orientation.x = 0.0
        first_pose.pose.orientation.y = 0.0
        first_pose.pose.orientation.z = 0.0
        first_pose.pose.orientation.w = 1.0
        first_goal.target_pose = first_pose
        
        try:
            self._client.send_goal(self._action_topic, first_goal)
            Logger.loginfo("Premier objectif envoyé à move_base : %s" % str(first_pose))
        except Exception as e:
            Logger.logwarn("Échec de l'envoi du premier objectif: %s" % str(e))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'
        
        current_time = rospy.Time.now()
        
        if self._step == 0:
            # Attente de l'arrivée au premier objectif
            if self._client.has_result(self._action_topic):
                Logger.loginfo("Premier objectif atteint.")
                self._step = 1
                self._timer_start = current_time
        
        elif self._step == 1:
            # Attente de 3 secondes après le premier objectif
            if (current_time - self._timer_start) >= rospy.Duration(3.0):
                # Envoi du second objectif provenant de l'input goal_pose_key
                second_goal = MoveBaseGoal()
                second_goal.target_pose = userdata.goal_pose_key
                try:
                    self._client.send_goal(self._action_topic, second_goal)
                    Logger.loginfo("Second objectif envoyé à move_base : %s" % str(userdata.goal_pose_key))
                except Exception as e:
                    Logger.logwarn("Échec de l'envoi du second objectif: %s" % str(e))
                    self._error = True
                    return 'failed'
                self._step = 2
        
        elif self._step == 2:
            # Attente de l'arrivée au second objectif
            if self._client.has_result(self._action_topic):
                Logger.loginfo("Second objectif atteint.")
                self._step = 3
                self._timer_start = current_time
        
        elif self._step == 3:
            # Attente de 3 secondes après le second objectif
            if (current_time - self._timer_start) >= rospy.Duration(3.0):
                # Efface le premier terme de la liste sur le serveur
                try:
                    requests.post(self.server_url + '/remove_first', timeout=1.0)
                    Logger.loginfo("Premier terme de la liste supprimé.")
                except Exception as e:
                    Logger.logwarn("Erreur lors du nettoyage de la liste: %s" % str(e))
                self._step = 4
                return 'done'
        
        return None

    def on_exit(self, userdata):
        # En cas de sortie prématurée, annulation de l'objectif actif
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Annulation de l'objectif move_base actif.")

