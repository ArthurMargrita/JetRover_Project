#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import requests

class OrderReceptionState(EventState):
    '''
    State qui interroge le serveur Flask pour récupérer la dernière commande.
    
    Il attend une commande sous la forme d'une chaîne, par exemple "1;une bière". 
    Si le numéro de table (la première partie avant le point-virgule) vaut "1",
    il crée une PoseStamped fixe (position : x=3, y=0, z=0 et orientation : 0,0,0,1), 
    efface la commande via l'endpoint /clear_list et renvoie l'outcome 'done'.
    
    Dans le cas contraire, il affiche un message indiquant que la commande est destinée 
    à une autre table et renvoie l'outcome 'failed'.
    
    Output Keys:
        goal_pose    geometry_msgs/PoseStamped
                     La position cible pour le robot (pour MoveBaseState).
    
    Outcomes:
        done    La commande concerne la table 1.
        failed  La commande concerne une table autre que 1.
    '''
    
    def __init__(self):
        super(OrderReceptionState, self).__init__(outcomes=['done', 'failed'],
                                                    output_keys=['goal_pose'])
        self.server_url = 'http://192.168.246.122:5000'  # À adapter si besoin
        self._order_received = False
        self._order = None

    def on_enter(self, userdata):
        self._order_received = False
        self._order = None
        Logger.loginfo("Attente de réception de commande...")

    def execute(self, userdata):
        try:
            # Récupère la dernière commande via l'endpoint /get_last
            response = requests.get(self.server_url + '/get_last', timeout=1.0)
            if response.status_code == 200:
                # La commande est attendue sous forme de chaîne, par exemple "1;une bière"
                order = response.json()  # order sera une chaîne de caractères
                if not order:
                    return None  # aucune commande disponible
                if not self._order_received:
                    self._order = order
                    self._order_received = True
                    
                    # On parse la commande en séparant sur le ';'
                    if ';' in order:
                        parts = order.split(';')
                    else:
                        parts = order.split()  # fallback si le ';' n'est pas présent
                    table_number = parts[0].strip()
                    
                    if table_number == '1':
                        # Crée le message PoseStamped avec la pose fixe
                        goal_pose = PoseStamped()
                        goal_pose.header.stamp = rospy.Time.now()
                        goal_pose.header.frame_id = "map"
                        goal_pose.pose.position = Point(3.0, 0.0, 0.0)
                        goal_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                        
                        userdata.goal_pose = goal_pose
                        Logger.loginfo("Commande pour table 1 reçue. Objectif : %s" % str(goal_pose))
                        
                        # Efface la commande en appelant /clear_list
                        try:
                            requests.post(self.server_url + '/clear_list', timeout=1.0)
                        except Exception as e:
                            Logger.logwarn("Impossible de nettoyer la liste de commandes: %s" % str(e))
                        
                        return 'done'
                    else:
                        Logger.logwarn("Commande reçue pour la table %s (attendu: 1)" % table_number)
                        return 'failed'
            else:
                Logger.logwarn("Aucune commande disponible (status %s)" % response.status_code)
        except Exception as e:
            Logger.logwarn("Erreur lors de la récupération de la commande: %s" % str(e))
        
        return None  # Continue d'attendre

    def on_exit(self, userdata):
        self._order_received = False

