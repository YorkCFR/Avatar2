import asyncio
import json
import pickle
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets

from avatar2_llm_bridge.llm_client import LocalLLM

_DEFAULT_STATIONS_JSON = json.dumps(
    # [{"avatar_name": "welcomeAvatar", "host": "192.168.0.114", "port": 5678}]
    # [{"avatar_name": "robotAvatar", "host": "192.168.0.114", "port": 5678}]
    [{"avatar_name": "printerAvatar", "host": "192.168.0.114", "port": 5678}]
)

class AvatarLLMBridge(Node):
    def __init__(self):
        super().__init__("avatar2_llm_bridge")

        self.declare_parameter("stations_json", _DEFAULT_STATIONS_JSON)
        stations_json = self.get_parameter("stations_json").value
        
        self.declare_parameter("chat_model_path", "/home/walleed/Documents/Avatar2/llms/Qwen3.6-35B-A3B-UD-Q8_K_XL.gguf")
        self._chat_model_path = self.get_parameter("chat_model_path").value
        
        self.declare_parameter("n_ctx", 4096)
        self._n_ctx = self.get_parameter("n_ctx").value
        
        self.declare_parameter("n_gpu_layers", -1)  # -1 = offload all layers to GPU
        self._n_gpu_layers = self.get_parameter("n_gpu_layers").value
        
        self.declare_parameter("enable_thinking", False)
        self._enable_thinking = self.get_parameter("enable_thinking").value
        
        self.declare_parameter("max_tokens", 512)
        self._max_tokens = self.get_parameter("max_tokens").value
        
        self.declare_parameter("temperature", 0.7)
        self._temperature = self.get_parameter("temperature").value
        
        self.declare_parameter("top_p", 0.8)
        self._top_p = self.get_parameter("top_p").value
        
        self.declare_parameter("sampling_top_k", 20)
        self._sampling_top_k = self.get_parameter("sampling_top_k").value
        
        self.declare_parameter("min_p", 0.0)
        self._min_p = self.get_parameter("min_p").value
        
        self.declare_parameter("presence_penalty", 1.5)
        self._presence_penalty = self.get_parameter("presence_penalty").value
        
        self.declare_parameter("repeat_penalty", 1.0)
        self._repeat_penalty = self.get_parameter("repeat_penalty").value
        
        self.declare_parameter(
            "system_prompt", "You are a friendly owl avatar. Answer briefly and warmly. "
            "Respond in plain text only - never use emojis, emoticons, "
            "or include images."
        )
        self._system_prompt = self.get_parameter("system_prompt").value
        
        self.declare_parameter("vectorstore_file", "")  # "" = no retrieval, otherwise a .pkl file containing a FAISS vectorstore
        self._vectorstore_file = self.get_parameter("vectorstore_file").value
        
        self.declare_parameter("retrieval_k", 3)
        self._retrieval_k = self.get_parameter("retrieval_k").value
        
        self.declare_parameter("session_enabled", False)  # False = no guided session, just answer at each station
        self._session_enabled = self.get_parameter("session_enabled").value
        
        self.declare_parameter("session_route", ["welcomeAvatar", "clockAvatar", "printerAvatar"])
        self._route = list(self.get_parameter("session_route").value)
        
        self.declare_parameter("session_leave_line", "Follow me, I will take you there.")
        self._leave_line = self.get_parameter("session_leave_line").value
        
        self.declare_parameter("session_arrive_line", "Here we are!")
        self._arrive_line = self.get_parameter("session_arrive_line").value
        
        self.declare_parameter("session_goodbye_line", "I am flying back to the entrance now. Goodbye.")
        self._goodbye_line = self.get_parameter("session_goodbye_line").value
        
        # added to the system prompt at the first station; a reply containing [GUIDE] starts a session. "" = by hand only
        self.declare_parameter(
            "session_guide_instruction",
            "If the person asks you to take them to a landmark or location, or to show or guide them there, "
            "reply with only [GUIDE] and nothing else."
        )
        self._guide_instruction = self.get_parameter("session_guide_instruction").value
        
        self.declare_parameter("session_waypoint_dwell", 8.0)  # seconds at each stop on the way
        self._waypoint_dwell = self.get_parameter("session_waypoint_dwell").value
        
        self.declare_parameter("session_idle_timeout", 60.0)  # seconds of silence at the last station before returning
        self._idle_timeout = self.get_parameter("session_idle_timeout").value
        
        self.declare_parameter("session_step_timeout", 20.0)  # longest wait for arrived / departed
        self._step_timeout = self.get_parameter("session_step_timeout").value
        
        self.declare_parameter("session_speech_start_timeout", 5.0)  # longest wait for the owl to start a line
        self._speech_start_timeout = self.get_parameter("session_speech_start_timeout").value

        try:
            self._stations = json.loads(stations_json)
        except Exception as e:
            raise ValueError(f"stations_json is not valid JSON: {stations_json!r} ({e})")
        if not isinstance(self._stations, list) or not self._stations:
            raise ValueError("stations_json must be a non-empty JSON list of station dicts")
        for s in self._stations:
            if not {"avatar_name", "host", "port"} <= s.keys():
                raise ValueError(
                    f"each station needs avatar_name/host/port, got {s!r}"
                )

        self._persona_by_avatar = {
            name: param.value
            for name, param in self.get_parameters_by_prefix("persona_by_avatar").items()
        }

        self._vectorstore = None
        if self._vectorstore_file:
            self.get_logger().info(f"loading vectorstore: {self._vectorstore_file}")
            with open(self._vectorstore_file, "rb") as f:
                self._vectorstore = pickle.load(f)

        # one owl is shared by the stations on the route; it starts at the first of them
        if self._session_enabled:
            known = {s["avatar_name"] for s in self._stations}
            if len(self._route) < 2 or not set(self._route) <= known:
                raise ValueError(
                    f"session_route needs at least two stations, all listed in stations_json, got {self._route!r}"
                )
        self._owl_at = self._route[0] if self._session_enabled else None  # None while the owl is on its way
        self._session_task = None
        self._status_waiters = []
        self._last_heard = 0.0

        self.get_logger().info(f"loading chat model: {self._chat_model_path}")
        self.llm = LocalLLM(
            model_path=self._chat_model_path,
            n_ctx=self._n_ctx,
            n_gpu_layers=self._n_gpu_layers,
            enable_thinking=self._enable_thinking,
            verbose=False,
        )
        
        for station in self._stations:
            avatar_name = station["avatar_name"]
            topic = f"/{avatar_name}/owl/prompt"
            self.create_subscription(
                String,
                topic,
                lambda msg, dest=avatar_name: self.on_manual_prompt(msg, dest),
                10,
            )
            self.get_logger().info(f"manual test topic for {avatar_name}: {topic}")

        if self._session_enabled:
            topic = f"/{self._route[0]}/owl/guide"
            self.create_subscription(String, topic, self.on_manual_guide, 10)
            self.get_logger().info(
                f"session on, route {' -> '.join(self._route)}, manual start topic: {topic}"
            )

        self._loop = None
        self._connections = {}
        self._ws_thread = threading.Thread(target=self._run_ws_clients, daemon=True)
        self._ws_thread.start()

        station_summary = ", ".join(
            f"{s['avatar_name']}@ws://{s['host']}:{s['port']}" for s in self._stations
        )
        self.get_logger().info(f"avatar2_llm_bridge ready - connecting to: {station_summary}")

    def on_manual_prompt(self, msg: String, dest: str):
        # manual testing: bypasses ava_bridge entirely, generate a reply for this station and log it; nothing is sent to the station
        reply = self.generate(msg.data, dest)
        self.get_logger().info(f"[manual:{dest}] {msg.data!r} -> {reply!r}")

    def on_manual_guide(self, msg: String):
        # starts a guided session by hand; called on ROS thread, so start is handed over to the websocket thread
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._start_session)

    def _retrieve(self, query: str) -> str:
        # closest paragraphs from the vectorstore; "" when retrieval is off or fails
        if self._vectorstore is None:
            return ""
        try:
            docs = self._vectorstore.similarity_search(query, k=self._retrieval_k)
        except Exception as e:
            self.get_logger().warn(f"retrieval failed, answering without context: {e}")
            return ""
        for doc in docs:
            self.get_logger().info(f"retrieved: {doc.page_content[:80]!r}")
        return "\n\n".join(doc.page_content for doc in docs)

    def generate(self, prompt: str, dest: str = None) -> str:
        # build the system prompt (persona, guide instruction, retrieval) and return the LLM's reply; called on the websocket thread, so it can block
        # dest selects a per-avatar persona override if one is configured;
        # falls back to the single system_prompt param otherwise
        system_prompt = self._persona_by_avatar.get(dest, self._system_prompt)
        if self._session_enabled and dest == self._route[0] and self._guide_instruction:
            system_prompt = f"{system_prompt}\n\n{self._guide_instruction}"
        context = self._retrieve(prompt)
        if context:
            system_prompt = (
                f"{system_prompt}\n\nUse the following information to answer. "
                f"If the answer is not in it, say you are not sure.\n\n{context}"
            )
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt},
        ]
 
        return self.llm.generate(
            messages,
            max_tokens=self._max_tokens,
            temperature=self._temperature,
            top_p=self._top_p,
            top_k=self._sampling_top_k,
            min_p=self._min_p,
            presence_penalty=self._presence_penalty,
            repeat_penalty=self._repeat_penalty,
        )
        
    def _run_ws_clients(self):
        # runs on its own thread: one asyncio loop for all the websocket clients, with a websocket client per station
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(
            asyncio.gather(*(self._client_loop(s) for s in self._stations))
        )

    async def _client_loop(self, station: dict):
        # stay connected to one station's ava_bridge and pass on each message; retry every 3s
        avatar_name = station["avatar_name"]
        uri = f"ws://{station['host']}:{station['port']}"
        while rclpy.ok():
            try:
                self.get_logger().info(f"[{avatar_name}] connecting to ava_bridge at {uri}")
                async with websockets.connect(uri) as ws:
                    self._connections[avatar_name] = ws
                    self.get_logger().info(f"[{avatar_name}] connected to ava_bridge")
                    async for raw in ws:
                        await self._on_ava_message(raw, avatar_name)
            except Exception as e:
                self.get_logger().warn(
                    f"[{avatar_name}] ava_bridge connection lost/failed: {e}, retrying in 3s"
                )
            finally:
                self._connections.pop(avatar_name, None)
            await asyncio.sleep(3)

    async def _on_ava_message(self, raw: str, expected_avatar_name: str):
        # handle a message from the ava_bridge at the given station, which is expected to be the one the owl is at
        # 'status' goes to the session, 'heard' is answered by the LLM with a 'say' command; or starts a session with a
        # reply containing [GUIDE] at the first station. expected_avatar_name is used to filter out messages from 
        # other stations, which are ignored
        try:
            package = json.loads(raw)
            cmd = package["cmd"]
            dest = package["dest"]
        except Exception as e:
            self.get_logger().warn(f"unparseable message from ava_bridge: {raw!r} ({e})")
            return
 
        if cmd == "status":
            if self._session_enabled and dest == expected_avatar_name:
                self._on_status(dest, package.get("arg", {}).get("text", ""))
            return
        if cmd != "heard":
            return  # only "heard" and "status" are relevant to this node right now
        if dest != expected_avatar_name:
            self.get_logger().warn(
                f"[{expected_avatar_name}] got message for unexpected dest {dest!r}, ignoring"
            )
            return
 
        text = package.get("arg", {}).get("text", "")
        if not text:
            return

        if self._session_enabled and dest in self._route:
            if dest != self._owl_at:
                self.get_logger().info(f"heard from {dest} but the owl is not there, ignoring: {text!r}")
                return
            self._last_heard = time.monotonic()
 
        self.get_logger().info(f"heard from {dest}: {text!r}")
        loop = asyncio.get_running_loop()
        reply = await loop.run_in_executor(None, self.generate, text, dest)
        if self._session_enabled and dest == self._route[0] and "[GUIDE]" in reply:
            self.get_logger().info(f"[{dest}] the person asked to be guided")
            self._start_session()
            return
        await self._send_say(dest, reply)
 
    async def _send_say(self, dest: str, text: str):
        # send a "say" command to the ava_bridge at the given station, if it is connected
        payload = json.dumps({"cmd": "say", "dest": dest, "args": {"text": text}})
        ws = self._connections.get(dest)
        if ws is not None:
            await ws.send(payload)
            self.get_logger().info(f"[{dest}] sent: {payload}")
        else:
            self.get_logger().warn(f"[{dest}] no active ava_bridge connection, dropping reply")

    async def _send_action(self, dest: str, act: str):
        # send an 'action' (fly_in, fly_away, wave, reset) command to the ava_bridge at the given station, if it is connected
        payload = json.dumps({"cmd": "action", "dest": dest, "args": {"act": act}})
        ws = self._connections.get(dest)
        if ws is None:
            self.get_logger().warn(f"[{dest}] no active ava_bridge connection, dropping {act}")
            return
        try:
            await ws.send(payload)
            self.get_logger().info(f"[{dest}] sent: {payload}")
        except Exception as e:
            self.get_logger().warn(f"[{dest}] could not send {act}: {e}")

    def _on_status(self, dest: str, text: str):
        # an owl reported what it is doing; wake any session step waiting for that report
        try:
            status = json.loads(text)
            key = (dest, status["cmd"], status["arg"])
        except Exception:
            return
        if key[1] == "talking" and key[2] in ("idle", "in_progress"):
            return  # heartbeats
        self.get_logger().info(f"status from {dest}: {key[1]}/{key[2]}")
        if dest == self._owl_at and key[1:] == ("talking", "over"):
            self._last_heard = time.monotonic()  # silence is counted from the end of the owl's reply
        for waiter in list(self._status_waiters):
            if waiter[0] == key and not waiter[1].done():
                waiter[1].set_result(True)

    def _expect(self, station: str, cmd: str, arg: str):
        # note that a status report is awaited; done before sending the command so a quick report is not missed
        waiter = ((station, cmd, arg), self._loop.create_future())
        self._status_waiters.append(waiter)
        return waiter

    async def _reached(self, waiter, timeout: float) -> bool:
        # wait for the given status to be reported, or timeout; returns True if it was reached, False if not
        try:
            await asyncio.wait_for(waiter[1], timeout)
            return True
        except asyncio.TimeoutError:
            return False
        finally:
            self._status_waiters.remove(waiter)

    async def _move(self, station: str, act: str, expect: str):
        # send a movement command and wait for the owl to confirm it; raises if it does not, which ends the session
        waiter = self._expect(station, "moving", expect)
        await self._send_action(station, act)
        if not await self._reached(waiter, self._step_timeout):
            raise RuntimeError(f"{station} did not report {expect} after {act}")

    async def _speak(self, station: str, text: str):
        # say a line and wait until the owl has finished it; a station that stays silent is skipped, not an error
        started = self._expect(station, "talking", "started")
        over = self._expect(station, "talking", "over")
        await self._send_say(station, text)
        if await self._reached(started, self._speech_start_timeout):
            await self._reached(over, 3 * self._step_timeout)
        else:
            self._status_waiters.remove(over)
            self.get_logger().warn(f"[{station}] did not start speaking, carrying on")

    def _start_session(self):
        # start a guided session unless one is already running; runs on the websocket thread
        if self._session_task is not None:
            self.get_logger().info("session: already running, ignoring start")
            return
        self._session_task = self._loop.create_task(self._run_session())

    async def _run_session(self):
        # one guided trip: leave the first station, visit each stop, talk at the last station, then return.
        # if a step is not confirmed, every station is reset
        home, stops, last = self._route[0], self._route[1:-1], self._route[-1]
        self.get_logger().info(f"session: started, guiding from {home} to {last}")
        try:
            self._owl_at = None  # nobody is answered while the owl is on its way
            await self._speak(home, self._leave_line)
            await self._move(home, "fly_away", "departed")
            for stop in stops:
                await self._move(stop, "fly_in", "arrived")
                await self._send_action(stop, "wave")
                await asyncio.sleep(self._waypoint_dwell)
                await self._move(stop, "fly_away", "departed")
            await self._move(last, "fly_in", "arrived")
            self._owl_at = last
            await self._speak(last, self._arrive_line)

            # stay for as long as the conversation keeps going
            self._last_heard = time.monotonic()
            while time.monotonic() - self._last_heard < self._idle_timeout:
                await asyncio.sleep(1.0)

            self.get_logger().info(f"session: quiet at {last} for {self._idle_timeout}s, returning to {home}")
            self._owl_at = None
            await self._speak(last, self._goodbye_line)
            await self._move(last, "fly_away", "departed")
            await self._move(home, "fly_in", "arrived")
        except Exception as e:
            # a step was not confirmed: put every screen back to how it starts, owl at the first station
            self.get_logger().warn(f"session: {e}; resetting all stations")
            for station in self._route:
                await self._send_action(station, "reset")
        finally:
            self._owl_at = home
            self._session_task = None
            self.get_logger().info(f"session: over, the owl is at {home}")

def main(args=None):
    rclpy.init(args=args)
    node = AvatarLLMBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # benign double-shutdown race with Ctrl-C's own signal handling
 
 
if __name__ == "__main__":
    main()