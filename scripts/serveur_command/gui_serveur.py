#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Server Control')
        self.setGeometry(300, 300, 500, 400)

        # Widgets
        self.start_btn = QtGui.QPushButton('Start Server', self)
        self.stop_btn = QtGui.QPushButton('Stop Server', self)
        self.show_list_btn = QtGui.QPushButton('Show List', self)
        self.log_display = QtGui.QTextEdit()
        self.log_display.setReadOnly(True)

        # Layout
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.show_list_btn)
        layout.addWidget(self.log_display)

        central_widget = QtGui.QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Connections
        self.start_btn.clicked.connect(self.start_server)
        self.stop_btn.clicked.connect(self.stop_server)
        self.show_list_btn.clicked.connect(self.show_order_list)

    def start_server(self):
        if not self.thread or not self.thread.isRunning():
            self.thread = ServerThread()
            self.thread.log_signal.connect(self.update_log)
            self.thread.start()
            self.update_log("Server started")

    def stop_server(self):
        try:
            req = urllib2.Request('http://localhost:5000/shutdown', data=urllib.urlencode({}))
            req.get_method = lambda: 'POST'
            urllib2.urlopen(req, timeout=1)
            self.update_log("Server stopped")
        except Exception as e:
            self.update_log("Shutdown error: {}".format(str(e)))

    def show_order_list(self):
        if self.thread and self.thread.server:
            orders = self.thread.server.order_list
            self.update_log("Current list: {}".format(orders))
        else:
            self.update_log("No active server")

    def update_log(self, message):
        # Convert to Unicode and handle encoding
        try:
            safe_msg = unicode(message, 'utf-8')
        except TypeError:
            safe_msg = unicode(message)
        
        # Format with Unicode objects
        timestamp = unicode(QtCore.QTime.currentTime().toString())
        log_entry = u"[{}] {}".format(timestamp, safe_msg)
        
        # Append to QTextEdit
        self.log_display.append(log_entry)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    win = ControlWindow()
    win.show()
    sys.exit(app.exec_())
