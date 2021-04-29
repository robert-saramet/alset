const express = require('express');
const app = express();
const net = require('net');
const path = require('path')

let clients = [];
let host = '127.0.0.1';
let port = 8080;
app.use(express.static('public'));
app.use(express.json());
app.listen(port, host, () => {console.log("listening on ", port)});
app.post('/api', (req, res) => {
	//res.sendFile("public/client.html");
	if(req.body.name == "rasppi") {
		// "Sign up" the rasppi
		clients.push({id: req.body.id, ip: req.body.ip, x: req.body.x, y: req.body.y});
		res.end();
	}
	else if (req.body.name == "verify_id") {
		const i = clients.findIndex(element => element.id == req.body.id);
		if (i == -1) {
			res.status(404).send('ID not found');
		}
		else {
			res.status(200).send({x: clients[i].x, y: clients[i].y});
		}
	}
	else {
		elem_index = clients.findIndex(element => element.id == req.body.name);
		if(elem_index != -1) {
			let elem = clients[elem_index];
			// Establish a connection between the server and the rasp pi via a socket
			let socket = net.Socket();
			socket.connect(8081, elem.ip)
			socket.write(JSON.stringify({command: {x: req.body.x, y: req.body.y}}))
			// "Sign out" the rasppi from the clients list
			clients.splice(elem_index, 1);
		}
		else { // Not really useful anymore
			// Emit error
			res.status(404).send('ID not found.');
		}
	}
});

