import websockets.sync.client
import time
import json

def get_packet(msg):
    """This makes many gross assumptions to return one word"""
    try:
        print("Getting a packet")
        parsed = json.loads(msg)
        q = parsed["arg"]
        z = str(q['text'])
        pp = json.loads(z)
        print(pp['description'])
        return pp['description']
    except:
        return '??'
  

uri = "ws://localhost:5678"
with websockets.sync.client.connect(uri) as websocket:
    for i in range(10000):
        print("Waiting for something....")
        msg = websocket.recv()
        q = get_packet(msg)
        while q != 'idle':
            print("Waiting for idle")
            msg = websocket.recv()
            q = get_packet(msg)
        websocket.send('{"cmd" : "say", "dest": "guy1", "args" :{"text":"this is the end"}}')
        while q != 'talking':
            print(f"Waiting for talking {q}")
            msg = websocket.recv()
            q = get_packet(msg)
        while q != 'idle':
            print(f"Waiting for idle {q}")
            msg = websocket.recv()
            q = get_packet(msg)


