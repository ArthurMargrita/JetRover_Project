#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flexbe_core import EventState, Logger

import requests

class CheckOrder_State_output(EventState):
	"""
	Une state qui vérifie si il y a des commandes.
	"""

	def __init__(self, ip_server_url):
		"""
		Constructeur de la state.
		"""
		super(CheckOrder_State_output, self).__init__(
			outcomes=['command', 'empty', 'failed'], 
			output_keys=['table', 'commande']
		)
		self.server_url = 'http://'+str(ip_server_url)+':5000'

	def execute(self, userdata):
		try:
			response = requests.get(self.server_url + '/get_first')

			if response.status_code == 200:
				commande_str = response.json()  # Ex: "4; 1 biere"
				try:
					table, commande = commande_str.split(";", 1)  # Séparer en 2 parties
					userdata.table = table.strip()
					userdata.commande = commande.strip()

					Logger.loginfo("Table: {}, Commande: {}".format(userdata.table, userdata.commande))
					return 'command'  # Modifier 'done' en 'command' pour correspondre aux outcomes

				except ValueError:
					Logger.logwarn("Erreur lors du découpage de la commande")
					return 'failed'

			elif response.status_code == 404:
				Logger.loginfo("Aucune commande en attente")
				return 'empty'

			else:
				Logger.logwarn("Erreur serveur: {}".format(response.status_code))
				return 'failed'

		except Exception as e:
			Logger.logerr("Exception lors de la récupération des commandes: {}".format(str(e)))
			return 'failed'
