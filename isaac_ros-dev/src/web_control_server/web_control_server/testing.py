import socketio

# Connect to the Flask-SocketIO server
sio = socketio.Client()

@sio.on('connect')
def on_connect():
    print("Connected to the server")

@sio.on('disconnect')
def on_disconnect():
    print("Disconnected from the server")

# Connect to your Flask-SocketIO server
server_url = "http://10.110.194.54:5002"
sio.connect(server_url)

# Simulate keypress events
keypress_data = [
    {'key': 'w', 'action': 'press'},
    {'key': 'w', 'action': 'release'},
    {'key': 's', 'action': 'press'},
    {'key': 's', 'action': 'release'},
    {'key': 'a', 'action': 'press'},
    {'key': 'a', 'action': 'release'},
]

for event in keypress_data:
    print(f"Sending keypress: {event}")
    sio.emit('keypress', event)

# Close the connection after testing
sio.disconnect()
