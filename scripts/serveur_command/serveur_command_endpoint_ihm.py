#!/usr/bin/env python
# encoding: utf-8

import os
import signal
import threading
from flask import Flask, request, jsonify

class FlaskServer:
    def __init__(self):
        self.app = Flask(__name__)
        self.order_list = []
        self.server_thread = None  # Référence pour garder le fil du serveur
        self.shutdown_event = threading.Event()  # Utilisé pour fermer le serveur proprement

        # Définition des routes de l'API Flask
        self.app.add_url_rule('/receive_data', 'receive_data', self.receive_data, methods=['POST'])
        self.app.add_url_rule('/get_last', 'get_last', self.get_last, methods=['GET'])
        self.app.add_url_rule('/get_first', 'get_first', self.get_first, methods=['GET'])
        self.app.add_url_rule('/get_all', 'get_all', self.get_all, methods=['GET'])
        self.app.add_url_rule('/clear_list', 'clear_list', self.clear_list, methods=['GET', 'POST'])
        self.app.add_url_rule('/remove_first', 'remove_first', self.remove_first, methods=['GET', 'POST'])
        self.app.add_url_rule('/remove_last', 'remove_last', self.remove_last, methods=['GET', 'POST'])

    def start_server(self):
        """Lance le serveur Flask dans un thread séparé."""
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def run_server(self):
        """Fonction pour exécuter le serveur Flask."""
        print("Serveur is running...")
        self.app.run(host='0.0.0.0', port=5000)

    def stop_server(self):
        """Arrête le serveur Flask proprement."""
        try:
            print("Attempting to stop server...")
            self.shutdown_event.set()  # Déclenche l'arrêt du serveur dans le thread
            self.server_thread.join()  # Attends que le thread de serveur se termine proprement
            print("Server stopped successfully")
        except Exception as e:
            print("Erreur lors de l'arrêt du serveur Flask : {}".format(e))

    def receive_data(self):
        """Reçoit des données et les ajoute à la liste des commandes."""
        data = request.get_data(as_text=True)
        if not data:
            return 'No data received', 400
        else:
            self.order_list.append(data)
            print("Order list:", self.order_list)
        return 'Data received successfully', 200

    def get_last(self):
        """Retourne le dernier élément de la liste des commandes."""
        if self.order_list:
            return jsonify(self.order_list[-1]), 200
        else:
            return 'Order list is empty', 404

    def get_first(self):
        """Retourne le premier élément de la liste des commandes."""
        if self.order_list:
            return jsonify(self.order_list[0]), 200
        else:
            return 'Order list is empty', 404

    def get_all(self):
        """Retourne tous les éléments de la liste des commandes."""
        if self.order_list:
            return jsonify(self.order_list), 200
        else:
            return 'Order list is empty', 404

    def clear_list(self):
        """Vide la liste des commandes."""
        self.order_list.clear()
        print("Order list cleared")
        return 'Order list cleared', 200

    def remove_first(self):
        """Supprime le premier élément de la liste des commandes."""
        if self.order_list:
            self.order_list.pop(0)
            print("Order list removed first")
            return 'Order list removed first', 200
        else:
            return 'Order list is empty', 404

    def remove_last(self):
        """Supprime le dernier élément de la liste des commandes."""
        if self.order_list:
            self.order_list.pop(-1)
            print("Order list removed last")
            return 'Order list removed last', 200
        else:
            return 'Order list is empty', 404


