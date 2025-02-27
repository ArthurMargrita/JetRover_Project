#!/usr/bin/env python
# encoding: utf-8

from flask import Flask, request, jsonify

app = Flask(__name__)

order_list = []

@app.route('/receive_data', methods=['POST'])
def receive_data():
    data = request.get_data(as_text=True)
    if not data:
        return 'No data received', 400
    else:
        order_list.append(data)
        print("Order list:", order_list)
    return 'Data received successfully', 200

@app.route('/get_last', methods=['GET'])
def get_last():
    if order_list:
        return jsonify(order_list[-1]), 200
    else:
        return 'Order list is empty', 404

@app.route('/get_first', methods=['GET'])
def get_first():
    if order_list:
        return jsonify(order_list[0]), 200
    else:
        return 'Order list is empty', 404

@app.route('/get_all', methods=['GET'])
def get_all():
    if order_list:
        return jsonify(order_list), 200
    else:
        return 'Order list is empty', 404

@app.route('/clear_list', methods=['GET', 'POST'])
def clear_list():
    order_list[:] = []
    print("Order list cleared")
    return 'Order list cleared', 200

@app.route('/remove_first', methods=['GET', 'POST'])
def remove_first():
    order_list.pop(0)
    print("Order list removed first")
    return 'Order list removed first', 200

@app.route('/remove_last', methods=['GET', 'POST'])
def remove_last():
    order_list.pop(-1)
    print("Order list removed last")
    return 'Order list removed last', 200



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

