#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class MoveState(EventState):
    '''
    Envoie un objectif de navigation au serveur move_base pour déplacer le robot vers la position donnée.
    
    Private Configuration:
        _text    (str) Chaîne de caractères au format "x,y,z,qx,qy,qz,qw" définissant le goal_pose.
    
    Output Keys:
        final_pose   geometry_msgs/PoseStamped
                     La dernière position connue du robot.
    
    Outcomes:
        done     Le robot a atteint la position demandée.
        failed   Un problème est survenu lors de l'envoi ou de l'exécution de l'objectif.
    
    Pendant l'exécution, la position actuelle du robot est affichée dans la console.
    '''
    def __init__(self, text=""):
        # Note : Nous n'utilisons plus d'input key pour le goal_pose.
        super(MoveState, self).__init__(outcomes=['done', 'failed'],
                                        output_keys=['final_pose'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._error = False
        
        # Paramètre privé contenant la chaîne de caractère définissant le goal_pose.
        self._text = text
        
        # Abonnement pour obtenir la position actuelle du robot (publié par AMCL dans RViz)
        self._current_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_cb)
    
    def _pose_cb(self, msg):
        # Conversion du message PoseWithCovarianceStamped en PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._current_pose = pose
    
    def on_enter(self, userdata):
        # Utilisation de self._text pour créer le goal_pose.
        if isinstance(self._text, str) and len(self._text) > 0:
            try:
                # On s'attend à une chaîne avec 7 valeurs séparées par des virgules : x,y,z,qx,qy,qz,qw
                values = [float(v.strip()) for v in self._text.split(",")]
                if len(values) != 7:
                    raise ValueError("La chaîne doit contenir 7 valeurs (x,y,z,qx,qy,qz,qw)")
                goal_pose = PoseStamped()
                goal_pose.header.stamp = rospy.Time.now()
                goal_pose.header.frame_id = "map"
                goal_pose.pose.position.x = values[0]
                goal_pose.pose.position.y = values[1]
                goal_pose.pose.position.z = values[2]
                goal_pose.pose.orientation.x = values[3]
                goal_pose.pose.orientation.y = values[4]
                goal_pose.pose.orientation.z = values[5]
                goal_pose.pose.orientation.w = values[6]
            except Exception as e:
                Logger.logwarn("Erreur lors de la conversion du goal_pose: %s" % str(e))
                return 'failed'
        else:
            Logger.logwarn("Aucun goal_pose fourni dans _text.")
            return 'failed'
        
        # Création du goal pour move_base
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
            userdata.final_pose = self._current_pose  # Stocke la dernière position connue
            return 'done'
        
        return None
    
    def on_exit(self, userdata):
        # En cas de sortie prématurée, annulation de l'objectif actif
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Annulation de l'objectif move_base actif.")

