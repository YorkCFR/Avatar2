"""Stand-ins for owl stations, to test the session logic of avatar2_llm_bridge without any station.

    python3 tools/fake_station.py welcomeAvatar:5678 clockAvatar:5679 printerAvatar:5680

Each name:port answers like that station's ava_bridge and its Unity owl: it reports arrived,
departed and the start and end of speech. A line typed here is sent to the LLM as speech heard
at the station where the owl currently is. The owl starts at the first station.
"""
import asyncio
import json
import sys

import websockets

stations = [(arg.split(":")[0], int(arg.split(":")[1])) for arg in sys.argv[1:]]
clients = {}
owl_at = [stations[0][0]]


def status(name, cmd, arg):
    return json.dumps({"cmd": "status", "dest": name, "arg": {"text": json.dumps({"cmd": cmd, "arg": arg})}})


def make_handler(name):
    async def handler(websocket):
        clients[name] = websocket
        print(f"[{name}] LLM connected", flush=True)
        try:
            async for raw in websocket:
                package = json.loads(raw)
                cmd, args = package["cmd"], package["args"]
                if cmd == "say":
                    print(f"[{name}] owl says: {args['text']}", flush=True)
                    await websocket.send(status(name, "talking", "started"))
                    await asyncio.sleep(3)
                    await websocket.send(status(name, "talking", "over"))
                elif cmd == "action":
                    print(f"[{name}] owl does: {args['act']}", flush=True)
                    if args["act"] == "fly_in":
                        await asyncio.sleep(3)
                        owl_at[0] = name
                        await websocket.send(status(name, "moving", "arrived"))
                    elif args["act"] == "fly_away":
                        await asyncio.sleep(3)
                        owl_at[0] = None
                        await websocket.send(status(name, "moving", "departed"))
                    elif args["act"] == "reset":
                        owl_at[0] = stations[0][0]
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            clients.pop(name, None)
            print(f"[{name}] LLM disconnected", flush=True)
    return handler


async def typed_lines():
    loop = asyncio.get_running_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            return
        text, name = line.strip(), owl_at[0]
        if not text:
            continue
        if name is None or name not in clients:
            print("the owl is not at a station right now, nothing sent", flush=True)
            continue
        print(f"[{name}] person says: {text}", flush=True)
        await clients[name].send(json.dumps({"cmd": "heard", "dest": name, "arg": {"text": text}}))


async def main():
    servers = [await websockets.serve(make_handler(name), "127.0.0.1", port) for name, port in stations]
    for name, port in stations:
        print(f"fake {name} on ws://127.0.0.1:{port}", flush=True)
    await typed_lines()
    await asyncio.Future()


if __name__ == '__main__':
    asyncio.run(main())
