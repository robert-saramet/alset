import requests
import socket
import json
import sys

def ConnectToServer(pos):
	id = sys.argv[1]
	server_host = '192.168.0.100'
	server_port = '8080'
	url = 'http://' + server_host + ':' + server_port + '/api'
	ip = socket.gethostbyname(socket.gethostname())
	port = 8081
	data = {
		"name": "rasppi",
		"id": id, # factory id (?)
		"ip": ip,
		"x": pos[0],
		"y": pos[1]
	}
	requests.post(url=url, json=data)
	
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.bind((ip, port))
	sock.listen()
	(conn, addr) = sock.accept()
	str = conn.recv(1024)
	return (json.loads(str)['command']['x'] - 1, json.loads(str)['command']['y'] - 1)
