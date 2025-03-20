#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flexbe_core import EventState, Logger
from time import sleep

class CheckPark(EventState):
    """
    State qui vérifie si le robot doit être en mode park ou non en fonction du numéro de table donné en entrée.
    """

    def __init__(self):
        """
        Constructeur de la state.
        """
        super(CheckPark, self).__init__(
            outcomes=['park', 'nonpark'],
            input_keys=['last_table']
        )

    def execute(self, userdata):
        """
        Vérifie si le robot doit entrer en mode park ou non.
        """
        if userdata.last_table == 0:
            sleep(10)
            return 'park'
        
        return 'nonpark'
