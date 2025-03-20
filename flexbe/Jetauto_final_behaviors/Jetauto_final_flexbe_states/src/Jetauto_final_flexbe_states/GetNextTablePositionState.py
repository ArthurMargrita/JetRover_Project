#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flexbe_core import EventState, Logger
import math
from time import sleep

class GetNextTablePositionState(EventState):
    """
    State qui renvoie la position de la table suivante en fonction du numéro de table donné en entrée.
    Si la table donnée est la dernière, renvoie la première pose pour faire une boucle.
    """

    def __init__(self, poses):
        """
        Constructeur de la state.
        :param poses: Liste de poses sous forme de [table, x, y, angle].
        Les poses vides seront ignorées.
        """
        super(GetNextTablePositionState, self).__init__(
            outcomes=['position', 'failed'],
            input_keys=['last_table'],
            output_keys=['x', 'y', 'table', 'angle']
        )

        # Filtrage des poses non vides
        self.poses = [pose for pose in poses if pose]  # Élimine les poses vides

    def execute(self, userdata):
        """
        Renvoie la position de la table suivante. Si last_table correspond à la dernière table, renvoie la première.
        """
        # Chercher la table correspondant à last_table
        for i, pose in enumerate(self.poses):
            if pose[0] == userdata.last_table:

                # Si on est à la dernière pose, renvoyer la première pose pour faire une boucle
                next_pose = self.poses[(i + 1) % len(self.poses)]
                # Définir les valeurs de sortie
                userdata.x = next_pose[1]
                userdata.y = next_pose[2]
                userdata.table = next_pose[0]
                userdata.angle = next_pose[3]

                Logger.loginfo("Table suivante trouvée: table={}, x={}, y={}, angle={}".format(
                    next_pose[0], next_pose[1], next_pose[2], next_pose[3]))
                return 'position'

        # Si aucune table ne correspond à last_table, renvoyer 'failed'
        Logger.logwarn("Numéro de table {} inconnu".format(userdata.last_table))
        return 'failed'

