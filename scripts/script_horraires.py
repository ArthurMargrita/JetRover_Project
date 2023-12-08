#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import datetime
from PyQt5.QtWidgets import (
	QApplication,
	QMainWindow,
	QVBoxLayout,
	QLineEdit,
	QLabel,
	QPushButton,
	QWidget,
	QHBoxLayout,
	QFormLayout,
)


class TimestampConverterApp(QMainWindow):
	def __init__(self):
		super(TimestampConverterApp, self).__init__()

		# Configuration de l'interface
		self.setWindowTitle("Convertisseur de Timestamp")
		self.setGeometry(100, 100, 400, 250)

		# Layout principal
		main_layout = QVBoxLayout()

		# Layout pour les champs de saisie
		form_layout = QFormLayout()

		# Champ de saisie du timestamp
		self.timestamp_input = QLineEdit()
		self.timestamp_input.setPlaceholderText("Entrez un timestamp Unix (ex: 1741008219.8747)")

		# Label pour afficher la conversion
		self.result_label = QLabel("Date et heure UTC : ")

		# Ajouter les champs au formulaire
		form_layout.addRow(QLabel("Timestamp Unix :"), self.timestamp_input)
		form_layout.addRow(QLabel("Résultat UTC :"), self.result_label)

		# Ajouter le formulaire au layout principal
		main_layout.addLayout(form_layout)

		# Layout pour les boutons
		button_layout = QHBoxLayout()

		# Bouton de conversion
		self.convert_button = QPushButton("Convertir")
		self.convert_button.clicked.connect(self.convert_timestamp)

		# Bouton pour afficher l'horodatage actuel
		self.current_time_button = QPushButton("Afficher l'horodatage actuel")
		self.current_time_button.clicked.connect(self.show_current_timestamp)

		# Ajouter les boutons au layout des boutons
		button_layout.addWidget(self.convert_button)
		button_layout.addWidget(self.current_time_button)

		# Ajouter le layout des boutons au layout principal
		main_layout.addLayout(button_layout)

		# Label pour afficher l'horodatage actuel du PC
		self.current_timestamp_label = QLabel("Horodatage actuel : ")
		main_layout.addWidget(self.current_timestamp_label)

		# Définir le widget principal
		container = QWidget()
		container.setLayout(main_layout)
		self.setCentralWidget(container)

	def convert_timestamp(self):
		"""
		Convertit un timestamp Unix en date et heure UTC.
		"""
		try:
			timestamp = float(self.timestamp_input.text())  # Convertit l'entrée en float
			date_time = datetime.datetime.utcfromtimestamp(timestamp)  # Conversion UTC
			self.result_label.setText("Date et heure UTC : {}".format(date_time))
		except ValueError:
			self.result_label.setText("Erreur : Entrez un timestamp valide.")

	def show_current_timestamp(self):
		"""
		Affiche l'horodatage Unix actuel du PC.
		"""
		current_timestamp = datetime.datetime.utcnow().timestamp()  # Timestamp actuel en UTC
		self.current_timestamp_label.setText("Horodatage actuel : {:.6f}".format(current_timestamp))



if __name__ == "__main__":
	try:
		app = QApplication(sys.argv)
		window = TimestampConverterApp()
		window.show()
		sys.exit(app.exec_())
	except KeyboardInterrupt:
		QApplication.instance().quit()

