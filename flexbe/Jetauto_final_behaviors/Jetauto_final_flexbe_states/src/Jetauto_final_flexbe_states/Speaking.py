#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import String

class Speaking(EventState):
    '''
    Ce state s'abonne au topic /speech et publie un message défini par le paramètre "data".

    -- data      string  Le message à publier sur /speech.
    -- duration  float   Temps d'attente après la publication avant de terminer l'état.

    <= done  Publication terminée.
    '''

    def __init__(self, data, duration=2.0):
        super(Speaking, self).__init__(outcomes=['done'])
        self.data = data
        self.duration = duration
        self.sub = None
        self.received_message = None

    def on_enter(self, userdata):
        # Initialisation du subscriber
        self.sub = rospy.Subscriber('/speech', String, self.callback)
        # Initialisation du publisher
        self.pub = rospy.Publisher('/speech', String, queue_size=10)
        Logger.loginfo("SpeechState initialisé avec message : '{}' et durée d'attente : {}s".format(self.data, self.duration))

    def callback(self, msg):
        """ Callback appelé lorsqu'un message est reçu sur /speech """
        self.received_message = msg.data
        Logger.loginfo("Message reçu: '{}'".format(self.received_message))

    def execute(self, userdata):
        """ Publie le message et termine l'état """
        if self.pub:
            self.pub.publish(String(self.data))
            Logger.loginfo("Message publié: '{}'".format(self.data))
            rospy.sleep(self.duration)  # Pause ajustable via le paramètre duration
            return 'done'

    def on_exit(self, userdata):
        """ Arrête le subscriber à la sortie de l'état """
        if self.sub is not None:
            self.sub.unregister()
            Logger.loginfo("Désabonnement du topic /speech")
