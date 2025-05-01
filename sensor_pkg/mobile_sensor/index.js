const https = require('https');
const fs = require('fs');
const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const zmq = require('zeromq');

const app = express();
let sensorPublisher, ttsSubscriber;
let ttsClients = new Set();

// Initialize ZMQ connections
async function initZMQ() {
  // Publisher for sensor data
  sensorPublisher = new zmq.Publisher();
  await sensorPublisher.bind('tcp://*:5556');
  console.log('Sensor ZMQ Publisher bound to port 5556');

  // Subscriber for TTS messages
  ttsSubscriber = new zmq.Subscriber();
  await ttsSubscriber.connect('tcp://localhost:5557');
  await ttsSubscriber.subscribe('tts');
  console.log('TTS ZMQ Subscriber connected to port 5557');

  // Handle incoming TTS messages
  for await (const [topic, message] of ttsSubscriber) {
    const text = message.toString();
    console.log('Received TTS message:', text);
    // Broadcast to all TTS WebSocket clients
    for (const client of ttsClients) {
      if (client.readyState === WebSocket.OPEN) {
        client.send(text);
      }
    }
  }
}
initZMQ().catch(err => console.error('Failed to initialize ZMQ:', err));

// Serve static files from the "public" folder
app.use(express.static('public'));
app.use(express.json());

// Serve index.html on GET /
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Endpoint to log pose data on the server (CLI)
app.post('/log', (req, res) => {
  console.log('Pose Data:', req.body);
  res.sendStatus(200);
});

const options = {
    key: fs.readFileSync('key.pem'),
    cert: fs.readFileSync('cert.pem'),
};

// Create HTTPS server
const server = https.createServer(options, app);

// Create WebSocket servers for both sensor data and TTS
const wss = new WebSocket.Server({ noServer: true });
const wssTTS = new WebSocket.Server({ noServer: true });

// Handle upgrade requests to separate WebSocket connections
server.on('upgrade', (request, socket, head) => {
  const pathname = new URL(request.url, `http://${request.headers.host}`).pathname;

  if (pathname === '/tts') {
    wssTTS.handleUpgrade(request, socket, head, (ws) => {
      wssTTS.emit('connection', ws, request);
    });
  } else {
    wss.handleUpgrade(request, socket, head, (ws) => {
      wss.emit('connection', ws, request);
    });
  }
});

// Handle sensor data WebSocket connections
wss.on('connection', (ws) => {
  console.log('New sensor data WebSocket client connected');
  
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (sensorPublisher) {
        await sensorPublisher.send(['sensor_data', JSON.stringify(data)]);
        // console.log('Data forwarded to ZMQ:', data.camera ? 'pose+camera' : 'pose only');
      }
    } catch (err) {
      console.error('Error processing message:', err);
    }
  });
});

// Handle TTS WebSocket connections
wssTTS.on('connection', (ws) => {
  console.log('New TTS WebSocket client connected');
  ttsClients.add(ws);
  
  ws.on('close', () => {
    ttsClients.delete(ws);
  });
});

const port = process.env.PORT || 4004;
server.listen(port, () => {
  console.log(`HTTPS server running at https://localhost:${port}`);
});

