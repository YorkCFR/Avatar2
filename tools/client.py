from websockets.sync.client import connect

uri = "ws://localhost:5678"
with connect(uri) as websocket:
    websocket.send('{"command" : "say", "argument" :{"text":"this is the end"}}')
    print(websocket.recv())
  
