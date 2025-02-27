#!/usr/bin/env python
# encoding: utf-8

from flask import Flask, request

app = Flask(__name__)

order_list = []

@app.route('/receive_data', methods=['POST'])
def receive_data():
    data = request.get_data(as_text=True)
    if not data:
        return 'No data received', 400
    else:
        order_list.append(data)
        print(order_list)
    
    return 'Data received successfully', 200
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # 0.0.0.0 permet d'écouter sur toutes les interfaces réseau

