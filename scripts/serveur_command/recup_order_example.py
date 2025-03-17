#!/usr/bin/env python
# encoding: utf-8

import requests


server_url = 'http://192.168.246.122:5000' # Adresse IP et port de votre serveur Flask





response = requests.get(server_url + '/get_all')
print(response.text.encode('utf-8').decode('utf-8'))


'''
response = requests.get(server_url + '/get_all')
print(response.text.encode('utf-8').decode('utf-8'))

response = requests.post(server_url + '/clear_list')
print(response.text.encode('utf-8').decode('utf-8'))

response = requests.get(server_url + '/get_all')
print(response.text.encode('utf-8').decode('utf-8'))

response = requests.get(server_url + '/remove_first')
print(response.text.encode('utf-8').decode('utf-8'))

response = requests.get(server_url + '/get_first')
print(response.text.encode('utf-8').decode('utf-8'))

'''


