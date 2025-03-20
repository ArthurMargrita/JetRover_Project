#!/usr/bin/env python
# encoding: utf-8

from flask import Flask, request, jsonify

class OrderServer:
	def __init__(self, log_callback=None):
		self.app = Flask(__name__)
		self.order_list = []
		self.log_callback = log_callback
		self.setup_routes()

	def log(self, message):
		if self.log_callback:
			self.log_callback(message)

	def setup_routes(self):
		self.app.add_url_rule('/receive_data', 'receive_data', self.receive_data, methods=['POST'])
		self.app.add_url_rule('/get_first', 'get_first', self.get_first, methods=['GET'])
		self.app.add_url_rule('/get_all', 'get_all', self.get_all, methods=['GET'])
		self.app.add_url_rule('/clear_list', 'clear_list', self.clear_list, methods=['GET', 'POST'])
		self.app.add_url_rule('/remove_first', 'remove_first', self.remove_first, methods=['GET', 'POST'])
		self.app.add_url_rule('/shutdown', 'shutdown', self.shutdown, methods=['POST'])

	def get_first(self):
		#self.log("Requesting first element")
		if self.order_list:
			return jsonify(self.order_list[0]), 200
		return 'Order list is empty', 404

	def remove_first(self):
		#self.log("Removing first element")
		if self.order_list:
			self.order_list.pop(0)
			return 'First item removed', 200
		return 'Order list is empty', 404

	def clear_list(self):
		#self.log("Clearing list")
		del self.order_list[:]
		return 'Order list cleared', 200

	def get_all(self):
		#self.log("Demande de toute la liste")
		if self.order_list:
			return jsonify(self.order_list), 200
		return 'Order list is empty', 404

	def receive_data(self):
		data = request.get_data(as_text=True)
		if not data:
			self.log("Tentative de réception de données vide")
			return 'No data received', 400
		self.order_list.append(data)
		self.log("Donnée reçue : {}".format(data))
		return 'Data received successfully', 200

	def shutdown(self):
		self.log("Arrêt du serveur initié")
		func = request.environ.get('werkzeug.server.shutdown')
		if func is None:
			raise RuntimeError('Not running with the Werkzeug Server')
		func()
		return 'Server shutting down...', 200

	def run(self, host='0.0.0.0', port=5000):
		self.log("Serveur démarré sur {}:{}".format(host, port))
		self.app.run(host=host, port=port)

