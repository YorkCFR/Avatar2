import websockets.sync.client
import time
import json

def get_packet(webosocket):
    """This makes many gross assumptions to return one word"""
    print("Getting a packet")
    msg = websocket.recv()
    print(msg)
    parsed = json.loads(msg)
    q = parsed["arg"]
    z = str(q['text'])
    pp = json.loads(z)
    print(pp['description'])
    return pp['description']
  

uri = "ws://localhost:5678"
with websockets.sync.client.connect(uri) as websocket:
    for i in range(10000):
        print("Waiting for something....")
        cmd = get_packet(websocket)
        while cmd != "idle":
            print("Waiting for idle")
            cmd = get_packet(websocket)
        print("No longer idle. Trying to send something")
        websocket.send('{"cmd" : "say", "dest": "guy1", "args" :{"text":"this is the end"}}')
        while cmd != "talking":
            print(f"Waiting for avatar to start talking {cmd}")
            cmd = get_packet(websocket)
        while cmd != "idle":
            print(f"Waiting for idle {cmd}")
            cmd = get_packet(websocket)

  
