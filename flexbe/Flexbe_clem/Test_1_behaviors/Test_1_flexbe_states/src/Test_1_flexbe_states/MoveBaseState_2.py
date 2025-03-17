#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class MoveBaseState_2(EventState):
    '''
    Envoie un objectif de navigation au serveur move_base pour déplacer le robot
    vers une destination prédéfinie en fonction d'un index donné.

    Input Keys:
        number    int
                  Un chiffre déterminant la destination :
                  0 -> Position: (-4, -1, 0) avec orientation (qx,qy,qz,qw) = (0, 0, 1, 0)
                  1 -> Position: (-3,  1, 0) avec orientation (qx,qy,qz,qw) = (0, 0, 0.7, -0.7)
                  2 -> Position: (-1,  1, 0) avec orientation (qx,qy,qz,qw) = (0, 0, 0.7, -0.7)
                  3 -> Position: ( 1,  1, 0) avec orientation (qx,qy,qz,qw) = (0, 0, 0.7, -0.7)
                  4 -> Position: (-1.5,-2.5, 0) avec orientation (qx,qy,qz,qw) = (0, 0, 0.7, -0.7)

    Output Keys:
        number    int
                  Le chiffre de destination utilisé, une fois la trajectoire complétée.

    Outcomes:
        done     Le robot a atteint la destination souhaitée.
        failed   Le chiffre donné en input n'est pas reconnu ou une erreur s'est produite lors de l'envoi de l'objectif.
    '''
    def __init__(self):
        super(MoveBaseState_2, self).__init__(outcomes=['done', 'failed'],
                                                input_keys=['number'],
                                                output_keys=['number'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._error = False
        self._goal_index = None
        self._goal_pose = None

        # Abonnement pour obtenir la position actuelle du robot (par exemple via AMCL)
        self._current_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_cb)

    def _pose_cb(self, msg):
        # Conversion du message PoseWithCovarianceStamped en PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._current_pose = pose

    def on_enter(self, userdata):
        goal_index = userdata.number
        self._goal_index = goal_index

        # Dictionnaire des destinations prédéfinies avec les nouvelles coordonnées et orientations.
        # Pour l'index 0 : rotation de 180° -> (0, 0, 1, 0)
        # Pour les indices 1 à 4 : rotation de 270° -> (0, 0, 0.7, -0.7)
        destinations = {
            0: [-4.0, -1.0, 0.0,  0.0, 0.0, 1.0, 0.0],
            1: [-3.0,  1.0, 0.0,  0.0, 0.0, 0.7, -0.7],
            2: [-1.0,  1.0, 0.0,  0.0, 0.0, 0.7, -0.7],
            3: [ 1.0,  1.0, 0.0,  0.0, 0.0, 0.7, -0.7],
            4: [-1.5, -2.5, 0.0,  0.0, 0.0, 0.7, -0.7]
        }

        if goal_index not in destinations:
            Logger.logwarn("La destination est inconnue")
            self._error = True
            return

        dest = destinations[goal_index]

        # Création du PoseStamped à partir de la destination choisie
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"  # Vérifiez que ce frame_id correspond à votre configuration
        pose.pose.position.x = dest[0]
        pose.pose.position.y = dest[1]
        pose.pose.position.z = dest[2]

        # Normalisation du quaternion
        qx = dest[3]
        qy = dest[4]
        qz = dest[5]
        qw = dest[6]
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm > 0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
        else:
            Logger.logwarn("Le quaternion de la destination %s n'est pas valide." % str(goal_index))
            self._error = True
            return

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self._goal_pose = pose

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self._error = False  # Réinitialisation de l'état d'erreur
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo("Objectif envoyé à move_base pour la destination %s : %s" % (str(goal_index), str(pose)))
        except Exception as e:
            Logger.logwarn("Échec de l'envoi de l'objectif move_base:\n%s" % str(e))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'
        
        if self._current_pose is not None:
            # Affichage de la position et de l'orientation (quaternion) actuelle du robot
            current_pos = self._current_pose.pose.position
            current_ori = self._current_pose.pose.orientation
            Logger.loginfo("Position actuelle du robot : x=%.2f, y=%.2f, z=%.2f" % (current_pos.x, current_pos.y, current_pos.z))
            Logger.loginfo("Orientation actuelle (quaternion) : x=%.2f, y=%.2f, z=%.2f, w=%.2f" % (current_ori.x, current_ori.y, current_ori.z, current_ori.w))
        else:
            Logger.loginfo("Position actuelle du robot non disponible.")

        # Vérification de la distance entre la position actuelle et la destination
        if self._current_pose is not None and self._goal_pose is not None:
            dx = self._goal_pose.pose.position.x - self._current_pose.pose.position.x
            dy = self._goal_pose.pose.position.y - self._current_pose.pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            Logger.loginfo("Distance à la destination : %.2f" % distance)
            if distance < 0.3:  # Seuil de distance pour considérer que le robot est arrivé
                userdata.number = self._goal_index
                return 'done'
        
        return None

    def on_exit(self, userdata):
        # En cas de sortie prématurée, annulation de l'objectif actif
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Annulation de l'objectif move_base actif.")

