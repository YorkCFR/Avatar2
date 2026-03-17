import websockets.sync.client
import asyncio
import time
import json

async def say(websocket, s):
    x = '{"cmd" : "say", "dest": "welcomeAvatar", "args" : {"text":"' + s + '"}}'
    print(f"Sending {x}")
#    websocket.send(x)


async def get_packet(websocket):
    """This makes many gross assumptions to return one word"""
    print("Getting a packet")
    msg = websocket.recv()
    print(msg)
    parsed = json.loads(msg)
    q = parsed["arg"]
    print(q)
    z = str(q['text'])
    print(z)
    pp = json.loads(z)
    print(json.dumps(z))
    print(pp['cmd'])
    print(pp['arg'])
    return pp['arg']


async def speaker(websocket):
    while True:
        print("Speaker")
        await say(websocket, "Hello nurse")
        await asyncio.sleep(10)
        
async def listener(websocket):
    async for message in websocket:
        print(f"Received {message}")

async def tester():
    print("tester")
    uri = "ws://localhost:5678"
    async with websockets.connect(uri) as websocket:
        await asyncio.gather(speaker(websocket), listener(websocket))

print("And away we go")
asyncio.run(tester())
