#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from flexbe_core import EventState, Logger

class IncrementModuloState(EventState):
    '''
    Attend 2 secondes, puis incrémente le chiffre reçu en input de 1 modulo 5.

    Input Keys:
        number    int
                  Le chiffre de départ.

    Output Keys:
        number    int
                  Le chiffre après incrémentation modulo 5.

    Outcomes:
        done      L'opération s'est bien déroulée.
    '''
    def __init__(self):
        super(IncrementModuloState, self).__init__(outcomes=['done'],
                                                     input_keys=['number'],
                                                     output_keys=['number'])
        self._start_time = None
        self._input_number = None

    def on_enter(self, userdata):
        # Sauvegarde du chiffre d'entrée et enregistrement du temps de démarrage
        self._input_number = userdata.number
        self._start_time = rospy.Time.now()
        Logger.loginfo("Démarrage de l'attente de 2 secondes avant incrémentation.")

    def execute(self, userdata):
        # Attente non bloquante de 2 secondes
        if rospy.Time.now() - self._start_time < rospy.Duration(2):
            return None

        # Calcul du nouveau chiffre : (+1 modulo 5)
        new_number = (self._input_number + 1) % 5
        userdata.number = new_number
        Logger.loginfo("Incrémentation terminée : %s -> %s" % (str(self._input_number), str(new_number)))
        return 'done'

    def on_exit(self, userdata):
        pass

