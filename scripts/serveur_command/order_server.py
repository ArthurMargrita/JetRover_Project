#!/usr/bin/env python
# encoding: utf-8

from flask import Flask, request, jsonify
import logging

class OrderServer:
    def __init__(self, log_callback=None):
        self.app = Flask(__name__)
        self.order_list = []
        self.log_callback = log_callback
        self.setup_routes()

    def log(self, message):
        # Envoie le message à la fois à la console et à l'interface
        logging.info(message)
        if self.log_callback:
            self.log_callback(message)

    def setup_routes(self):
        routes = [
            ('/receive_data', 'receive_data', self.receive_data, ['POST']),
            ('/get_last', 'get_last', self.get_last, ['GET']),
            ('/get_first', 'get_first', self.get_first, ['GET']),
            ('/get_all', 'get_all', self.get_all, ['GET']),
            ('/clear_list', 'clear_list', self.clear_list, ['GET', 'POST']),
            ('/remove_first', 'remove_first', self.remove_first, ['GET', 'POST']),
            ('/remove_last', 'remove_last', self.remove_last, ['GET', 'POST']),
            ('/shutdown', 'shutdown', self.shutdown, ['POST'])
        ]
        
        for route in routes:
            self.app.add_url_rule(*route)

    def receive_data(self):
        data = request.get_data(as_text=True)
        if not data:
            self.log("Tentative de réception de données vide")
            return 'No data received', 400
        
        self.order_list.append(data)
        self.log("Données reçues : {} | Liste : {} éléments".format(data, len(self.order_list)))
        return 'Data received successfully', 200

    # Les autres méthodes restent similaires mais utilisent self.log() au lieu de print()

    def clear_list(self):
        self.order_list.clear()
        self.log("Liste vidée")
        return 'Order list cleared', 200

    def shutdown(self):
        self.log("Arrêt du serveur demandé")
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            self.log("Erreur d'arrêt : environnement Werkzeug non détecté")
            raise RuntimeError('Not running with the Werkzeug Server')
        func()
        return 'Server shutting down...', 200

    def run(self, host='0.0.0.0', port=5000):
        self.log("Démarrage du serveur sur {}:{}".format(host, port))
        self.app.run(host=host, port=port)

if __name__ == '__main__':
    server = OrderServer()
    server.run()
