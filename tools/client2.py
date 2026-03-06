from websockets.sync.client import connect
import time

uri = "ws://localhost:5678"
with connect(uri) as websocket:
    for i in range(10000):
#        websocket.send('{"cmd" : "say", "dest": "guy1", "args" :{"text":"this is the end"}}')
        print("Waiting for something....")
        print(websocket.recv())

        time.sleep(1)
  
