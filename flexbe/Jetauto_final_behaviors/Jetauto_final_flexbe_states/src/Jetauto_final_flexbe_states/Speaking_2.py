#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import String

class Speaking_2(EventState):
    '''
    Ce state publie un message construit à partir des entrées command et table.

    -- texte     string  Format du message à publier, doit inclure {command} et {table}.
    -- pause     float   Temps d'attente après la publication avant de terminer l'état.
    
    ># command   string  Commande à insérer dans le message.
    ># table     string  Numéro ou nom de la table.
    
    <= done  Publication terminée.
    '''

    def __init__(self, texte, pause=2.0):
        super(Speaking_2, self).__init__(outcomes=['done'], input_keys=['command', 'table'])
        self.texte = texte
        self.pause = pause
        self.pub = None
    
    def on_enter(self, userdata):
        self.pub = rospy.Publisher('/speech', String, queue_size=10)
        message = self.texte.format(command=userdata.command, table=userdata.table)
        self.pub.publish(String(message))
        Logger.loginfo("Message publié: '{}'".format(message))
    
    def execute(self, userdata):
        rospy.sleep(self.pause)  # Pause ajustable via le paramètre pause
        return 'done'

