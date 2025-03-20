#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from flexbe_core import EventState, Logger

class SendNumberState(EventState):
    '''
    Envoie en output le numéro passé en paramètre dans l'outcome done.

    Input Parameters:
        number    int
                  Le numéro à envoyer.

    Output Keys:
        number    int
                  Le numéro passé en paramètre.

    Outcomes:
        done      L'opération s'est bien déroulée.
    '''
    def __init__(self, last_table):
        super(SendNumberState, self).__init__(outcomes=['done'],
                                              output_keys=['last_table'])
        self._number = last_table

    def execute(self, userdata):
        userdata.last_table = self._number
        Logger.loginfo("Le chiffre {} a été envoyé.".format(self._number))
        return 'done'
