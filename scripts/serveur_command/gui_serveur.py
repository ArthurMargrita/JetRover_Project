#!/usr/bin/env python
# encoding: utf-8

import sys
import urllib
import urllib2
from PyQt4 import QtGui, QtCore
from order_server import OrderServer

class ServerThread(QtCore.QThread):
    log_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super(ServerThread, self).__init__()
        self.server = None

    def run(self):
        self.server = OrderServer(log_callback=self.handle_log)
        self.server.run()

    def handle_log(self, message):
        self.log_signal.emit(message)

class ControlWindow(QtGui.QMainWindow):
    def __init__(self):
        super(ControlWindow, self).__init__()
        self.thread = None
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Contrôle du Serveur')
        self.setGeometry(300, 300, 600, 400)

        # Widgets
        self.start_btn = QtGui.QPushButton('Démarrer le serveur', self)
        self.stop_btn = QtGui.QPushButton('Arrêter le serveur', self)
        self.get_list_btn = QtGui.QPushButton('Afficher la liste', self)
        self.status_label = QtGui.QLabel('Statut : Serveur arrêté')
        self.log_area = QtGui.QTextEdit()
        self.log_area.setReadOnly(True)

        # Disposition
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.get_list_btn)
        layout.addWidget(self.status_label)
        layout.addWidget(QtGui.QLabel("Journal d'événements :"))
        layout.addWidget(self.log_area)

        central_widget = QtGui.QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Connexions
        self.start_btn.clicked.connect(self.start_server)
        self.stop_btn.clicked.connect(self.stop_server)
        self.get_list_btn.clicked.connect(self.show_order_list)

    def start_server(self):
        if not self.thread or not self.thread.isRunning():
            self.thread = ServerThread()
            self.thread.log_signal.connect(self.update_log)
            self.thread.start()
            self.status_label.setText('Statut : Serveur actif')
            self.log_area.append("=== Serveur démarré ===")

    def stop_server(self):
        try:
            req = urllib2.Request('http://localhost:5000/shutdown', data=urllib.urlencode({}))
            req.get_method = lambda: 'POST'
            urllib2.urlopen(req, timeout=1)
            self.status_label.setText('Statut : Serveur arrêté')
        except Exception as e:
            self.log_area.append("Erreur d'arrêt : {str()}".format(e))
            QtGui.QMessageBox.warning(self, 'Erreur', str(e))

    def show_order_list(self):
        if self.thread and self.thread.server:
            orders = self.thread.server.order_list
            msg = "Éléments dans la liste : {}\n{}".format(len(orders),orders)
            QtGui.QMessageBox.information(self, 'Liste des commandes', msg)
        else:
            QtGui.QMessageBox.warning(self, 'Erreur', 'Serveur non démarré')

    def update_log(self, message):
        self.log_area.append(message)
        # Défilement automatique vers le bas
        scrollbar = self.log_area.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        if self.thread and self.thread.isRunning():
            self.stop_server()
            self.thread.quit()
            self.thread.wait(2000)
        event.accept()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    win = ControlWindow()
    win.show()
    sys.exit(app.exec_())
