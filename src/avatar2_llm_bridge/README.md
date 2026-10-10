This node is the shared LLM the owl avatars. The node loads the chat model once and connects to each
stations `ava_bridge` over a WebSocket. It answers `heard` messages with say` replies, optionally using 
retrieved library knowledge. With a guided session enabled, it also sends movement commands and tracks 
which station the owl occupies.

`avatar2_llm_bridge_node.py` handles ROS, retrieval and the station/session logic. `llm_client.py` loads 
the model and formats its input before generating a reply. The station's text-to-speech and Unity app 
turn the reply and movement commands into speech and animation. 

## Launch Files and Configs
| Launch file | Parameter files | Behavior |
| --- | --- | --- |
| `owl.llm.launch.py` | `config/owl_llm_params.yaml` | Answer questions at the configured stations. Guided sessions remain off through the node's default `session_enabled: False`. |
| `owl.session.launch.py` | Base file, then `config/owl_session_params.yaml` | Use the same model and knowledge, with the guided session enabled. |

The session launch passes `parameters=[config, session_config]`. A value in the session file overrides 
the same parameter in the base file; parameters omitted from both use the defaults declared in the node.
Parameters (config/owl_llm_params.yaml)
* stations_json - the stations to connect to, one entry per ava_bridge passed as a json list
* chat_model_path - the gguf chat model
* vectorstore_file - the pkl built by tools/ingest_data.py. Set it to "" to answer without retrieval
* retrieval_k - how many paragraphs to fetch for each question
* rebuild the package after editing the file, unless the workspace was built with --symlink-install

Installation Notes

The node runs in a python virtual environment that can also see the system packages ROS uses. 
When used on the lab equipment, use the following:

    python3 -m venv --system-site-packages ~/venvs/avatar_llm
    source ~/venvs/avatar_llm/bin/activate

This requires
* llama-cpp-python built with CUDA
* websockets (pip install websockets)
* for retrieval (pip install "numpy==1.26.4" "langchain==0.3.30" "langchain-community==0.3.31" "sentence-transformers==6.1.0" "faiss-cpu==1.15.1")

Keep numpy at the system version. ROS uses it too. tools/ingest_data.py does not run on langchain 1.x.
Build the pkl in this same environment. See scenarios/library_main/README.md

## Running the node from the virtual environment

colcon makes the installed node start with the system python, which cannot see the packages in the virtual
environment. setup.cfg therefore contains

    [build_scripts]
    executable=/usr/bin/env python3

so the node starts with whichever python3 is active. Activate the virtual environment in every terminal before running the node

    source ~/venvs/avatar_llm/bin/activate
    source install/setup.bash
    ros2 launch avatar2_llm_bridge owl.llm.launch.py

To check, look at the first line of the installed node. It should read #!/usr/bin/env python3

    head -1 install/avatar2_llm_bridge/lib/avatar2_llm_bridge/avatar2_llm_bridge_node

The setting only takes effect with a normal colcon build. With --symlink-install the first line stays
#!/usr/bin/python3 and the node stops with ModuleNotFoundError: No module named 'websockets'. In that case add the
virtual environment to the python path in each terminal instead

    export PYTHONPATH=$PYTHONPATH:$VIRTUAL_ENV/lib/python3.12/site-packages

After building the package and source or configuration edits:
 * For ordinary, non-guided replies:
 ```bash
  ros2 launch avatar2_llm_bridge owl.llm.launch.py
 ```

 * For guided sessions:
 ```bash
  ros2 launch avatar2_llm_bridge owl.session.launch.py
 ```
 
 * For inspecting param values:
 ```bash 
  ros2 param dump /avatar2_llm_bridge
 ```

## Guided session

The owl is on one screen at a time. With session_enabled the node keeps track of where the owl is and only answers
speech from that station. A session takes the owl from the first station on session_route to the last one and back

* it says session_leave_line and flies away from the first station
* it flies in, waves and flies away again at each stop on the way, staying session_waypoint_dwell seconds
* it flies in at the last station, says session_arrive_line and carries on the conversation there
* after session_idle_timeout seconds of silence it says session_goodbye_line and flies back

The node sends "action" messages (fly_in, fly_away, wave, reset) and waits for the "status" messages each owl
sends back (arrived, departed, the start and end of speech). The ava_bridge of every station on the route must run
with status_cmd set to status. If a station does not report within session_step_timeout seconds the node resets
all stations, which puts the owl back at the first one.

Parameters (config/owl_session_params.yaml)
* stations_json - must list every station on the route
* session_enabled - False leaves the node as it is without the session
* session_route - the stations the owl visits, in order
* session_leave_line, session_arrive_line, session_goodbye_line - what the owl says on the way
* session_guide_instruction - added to the system prompt at the first station. When the reply contains [GUIDE] the
  node starts a session instead of speaking the reply. Set it to "" to start sessions by hand only
* session_waypoint_dwell, session_idle_timeout, session_step_timeout - the timings, in seconds

A session starts when the person at the first station asks to be taken to the printer. It can also be started by
hand

    ros2 topic pub --once /welcomeAvatar/owl/guide std_msgs/msg/String "{data: 'go'}"

tools/fake_station.py stands in for the stations, so the session can be tried on one machine

    python3 tools/fake_station.py welcomeAvatar:5678 clockAvatar:5679 printerAvatar:5680

## Testing without a station

Each station has a test topic. The reply appears in the log of the node

    ros2 topic pub --once /printerAvatar/owl/prompt std_msgs/msg/String "{data: 'How much does 3D printing cost?'}"

With retrieval on, the log shows a "retrieved:" line for each paragraph that was used.

There is a tool for stand-in stations `fake_stations.py` 
In Terminal A:
```bash
python3 tools/fake_station.py welcomeAvatar:5678 clockAvatar:5679 printerAvatar:5680
```
followed by in Terminal B launching `owl.session.launch.py`. Wait for all three `connected to ava_bridge` lines,
then enter questions in terminal A:

| Input | Expected behavior |
| --- | --- |
| `What are your opening hours` | An ordinary reply at Welcome, without starting a trip. |
| `Can you take me to the 3D printer` | Leave announcement, departure from Welcome, Clock visit, then arrival and greeting at Printer. |
| `How much does 3D printing cost` | A reply at Printer using retrieved knowledge. |
| No input for about a minute after the reply ends | Goodbye at Printer, departure, then arrival back at Welcome. |

This scenario is tied to the library scenario

A trip can also be started manually, after the bridges connect:

```bash
ros2 topic pub --once /welcomeAvatar/owl/guide std_msgs/msg/String "{data: 'go'}"
```

For an isolated reply test, each configured station has a `/<station>/owl/prompt` topic:

```bash
ros2 topic pub --once /printerAvatar/owl/prompt std_msgs/msg/String "{data: 'How much does 3D printing cost?'}"
```
