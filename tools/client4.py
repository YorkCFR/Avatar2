# WebSocket client with separate send and receive loops
import asyncio
import websockets
import json
from datetime import datetime

class WebSocketClient:
    """WebSocket client with bidirectional message handling"""

    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.running = False
        # Queue for outgoing messages
        self.send_queue = asyncio.Queue()

    async def connect(self):
        """Establish WebSocket connection"""
        self.websocket = await websockets.connect(self.uri)
        self.running = True
        print(f"Connected to {self.uri}")

    async def disconnect(self):
        """Close the WebSocket connection gracefully"""
        self.running = False
        if self.websocket:
            await self.websocket.close()
            print("Disconnected")

    async def send_message(self, message: dict):
        """Queue a message for sending"""
        await self.send_queue.put(message)

    async def _sender(self):
        """Task that sends queued messages to the server"""
        while self.running:
            try:
                # Wait for message from queue with timeout
                message = await asyncio.wait_for(
                    self.send_queue.get(),
                    timeout=1.0
                )
                # Serialize and send the message
                await self.websocket.send(json.dumps(message))
                print(f"Sent: {message}")
            except asyncio.TimeoutError:
                # No message in queue, continue loop
                continue
            except websockets.ConnectionClosed:
                print("Connection closed while sending")
                break

    async def _receiver(self):
        """Task that receives and processes messages from the server"""
        while self.running:
            try:
                # Wait for incoming message
                message = await self.websocket.recv()
                print(f"Got message {message}")
                data = json.loads(message)
                await self._handle_message(data)
            except websockets.ConnectionClosed:
                print("Connection closed while receiving")
                break

    async def _process_payload(self, data: dict):
#       assuming all commands are status commands
        try:
            payload = json.loads(str(arg['text']))
            payload_cmd = payload['cmd']
            payload_arg = payload['arg']
        except:
            print("Could not parse payload")
            payload_cmd = '??'
            payload_arg = '??'

        if payload_cmd == 'talking':
            print(f"owl talking status is {payload['arg']}")
            if payload_arg == 'idle':
                status = 'idle'
            elif payload_arg == 'nominal':
                status = 'talking'
            elif payload_arg == 'waiting to talk':
                status = 'idle'
            else:
                status = 'idle'
            print(status)
            return status
        return "no idea"

    async def _handle_message(self, data: dict):
        """Process received messages based on type"""
        print(f"Got a message: {data}")
        cmd = data['cmd']
        avatar = data['dest']
        arg = data["arg"]

        print(f"Processing message from {avatar} command {cmd}")
        print(arg['text'])

        state = self._process_payload(arg)
        if state == 'idle':
            s = "polly wants a cracker"
            x = '{"cmd" : "say", "dest": "welcomeAvatar", "args" : {"text":"' + s + '"}}'
            print(f"Sending {x}")
            self.send_message(x)
            print("sent")




    async def run(self):
        """Main entry point - connects and runs send/receive tasks"""
        await self.connect()

        # Run sender and receiver concurrently
        sender_task = asyncio.create_task(self._sender())
        receiver_task = asyncio.create_task(self._receiver())

        # Wait for both tasks (they run until connection closes)
        await asyncio.gather(sender_task, receiver_task)


async def main():
    client = WebSocketClient("ws://localhost:5678")

    # Start the client in background
    client_task = asyncio.create_task(client.run())

    while True:
        print("alive")
        await asyncio.sleep(10)
    

asyncio.run(main())

