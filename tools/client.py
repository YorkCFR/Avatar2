from websockets.sync.client import connect
import time

uri = "ws://localhost:5678"
with connect(uri) as websocket:
    for i in range(10):
        websocket.send('{"cmd" : "say", "dest": "welcomeAvatar", "args" : {"text":"this is the end"}}')
        print("Waiting for something....")
        print(websocket.recv())

#        time.sleep(5)
#        websocket.send('{"cmd" : "say", "dest": "welcomeAvatar", "args" : {"text":"this is the end, my friend"}}')
  
