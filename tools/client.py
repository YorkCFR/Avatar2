from websockets.sync.client import connect
import time

def say(s):
    x = '{"cmd" : "say", "dest": "welcomeAvatar", "args" : {"text":"' + s + '"}}'
    print(x)
    websocket.send(x)

uri = "ws://localhost:5678"
with connect(uri) as websocket:
    for i in range(1):
        say("This is the end, my friend run " + str(i))
#        say("This is the end, my friend. run " + str(i) + " The time has come the walrus said to talk of many things.Of shows and ships and sealing wax and cabbages and kings. And why the sea is boiling hot and whether pigs have wings.")
#        say('Hello Heath. This is a really long utterance from the owl which I am using to test the animation process. Its also a test of the ability of the system to deal with very long messages.')
        print("Waiting for something....")
#        print(websocket.recv())

        time.sleep(10)
#        websocket.send('{"cmd" : "say", "dest": "welcomeAvatar", "args" : {"text":"this is the end, my friend"}}')
  
