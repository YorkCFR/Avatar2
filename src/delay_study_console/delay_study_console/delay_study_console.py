#!/usr/bin/env python3
"""
Operator Console for LLM-Avatar Delay Management Experiments (Study 1 & Study 2)
Runs on Ubuntu 22.04 / ROS2 / Python 3.10. Communicates with Unity via rosbridge.
"""

import csv
import json
import os
import random
import time
import tkinter as tk
from tkinter import messagebox, ttk
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# --- Data Models ---

@dataclass
class Study1Trial:
    trial_num: int
    is_practice: bool
    delay_category: str       # 'short', 'medium', 'long'
    presentation_method: str  # 'baseline', 'caching', 'combined_obscuring'
    query_id: int
    query_text: str
    response_text: str
    recorded_latency_ms: int
    enforced_latency_ms: int  # ~10 ms for caching, else recorded_latency_ms
    
@dataclass
class Study2Trial:
    trial_num: int
    pair_id: int
    modality_a: str           # 'NF', 'V', 'G', 'VG'
    modality_b: str
    delay_ms: int = 5000      # Fixed 5s delay


# --- Schedule Generators ---

# def generate_study1_schedule(stimuli_by_cat: Dict[str, List[dict]]) -> List[Study1Trial]:
#     """Generates 2 practice trials + 45 randomized Study 1 trials (3x3 x 5 reps)."""
#     methods = ["baseline", "caching", "combined_obscuring"]
#     categories = ["short", "medium", "long"]

#     trials: List[Study1Trial] = []
#     # Pick 15 unique queries per delay category (3 methods * 5 reps)
#     for cat in categories:
#         pool = random.sample(stimuli_by_cat[cat], 15)
#         idx = 0
#         for method in methods:
#             for _ in range(5):
#                 stim = pool[idx]
#                 idx += 1
#                 rec_ms = int(stim["latency_ms"])
#                 enf_ms = 10 if method == "caching" else rec_ms
#                 trials.append(Study1Trial(
#                     trial_num=0,
#                     is_practice=False,
#                     delay_category=cat,
#                     presentation_method=method,
#                     query_id=stim["query_id"],
#                     query_text=stim["query_text"],
#                     recorded_latency_ms=rec_ms,
#                     enforced_latency_ms=enf_ms,
#                 ))

#     random.shuffle(trials)
#     for i, t in enumerate(trials, start=1):
#         t.trial_num = i

#     # Prepend 2 practice trials (not logged to main dataset)
#     practice = [
#         Study1Trial(-1, True, "short", "baseline", 0, "Practice: What is the weather like today?", 1200, 1200),
#         Study1Trial(-2, True, "medium", "combined_obscuring", 0, "Practice: How does a compass work?", 3500, 3500),
#     ]
#     return practice + trials

def generate_study1_schedule(stimuli_by_cat: Dict[str, List[dict]]) -> List[Study1Trial]:
    """Generates 2 practice trials + 45 randomized Study 1 trials (3x3 x 5 reps)."""
    methods = ["baseline", "caching", "combined_obscuring"]
    categories = ["short", "medium", "long"]

    trials: List[Study1Trial] = []
    for cat in categories:
        available = stimuli_by_cat[cat]
        if len(available) >= 15:
            pool = random.sample(available, 15)
        elif len(available) >= 5:
            pool = random.sample(available, len(available)) + random.sample(available, 15 - len(available))
        else:
            pool = random.choices(available, k=15)

        idx = 0
        for method in methods:
            for _ in range(5):
                stim = pool[idx]
                idx += 1
                rec_ms = int(stim["latency_ms"])
                enf_ms = 10 if method == "caching" else rec_ms
                trials.append(Study1Trial(
                    trial_num=0,
                    is_practice=False,
                    delay_category=cat,
                    presentation_method=method,
                    query_id=stim["query_id"],
                    query_text=stim["query_text"],
                    response_text=stim["response_text"],
                    recorded_latency_ms=rec_ms,
                    enforced_latency_ms=enf_ms,
                ))

    random.shuffle(trials)
    for i, t in enumerate(trials, start=1):
        t.trial_num = i

    practice = [
        Study1Trial(-1, True, "short", "baseline", 0, "Practice: What is the weather like today?", "Today is sunny.", 1200, 1200),
        Study1Trial(-2, True, "medium", "combined_obscuring", 0, "Practice: How does a compass work?", "A compass works by pointing to the magnetic north.", 3500, 3500),
    ]
    return practice + trials


def generate_study2_schedule() -> List[Study2Trial]:
    """Generates 24 randomized Study 2 paired-comparison trials (6 pairs x 2 orders x 2 reps)."""
    base_pairs = [
        (1, "NF", "V"),
        (2, "NF", "G"),
        (3, "NF", "VG"),
        (4, "V", "G"),
        (5, "V", "VG"),
        (6, "G", "VG"),
    ]
    trials: List[Study2Trial] = []
    for pair_id, m1, m2 in base_pairs:
        for _ in range(2):  # 2 repetitions per ordered pair
            trials.append(Study2Trial(0, pair_id, m1, m2))
            trials.append(Study2Trial(0, pair_id, m2, m1))

    random.shuffle(trials)
    for i, t in enumerate(trials, start=1):
        t.trial_num = i
    return trials


# --- ROS2 Node ---

class ExperimentRosNode(Node):
    def __init__(self, on_avatar_status_cb):
        super().__init__("operator_console_node")
        self.cmd_pub = self.create_publisher(String, "/experiment/command", 10)
        self.meta_pub = self.create_publisher(String, "/experiment/trial_meta", 10)
        self.status_sub = self.create_subscription(
            String, "/avatar/status", self._status_callback, 10
        )
        self.on_avatar_status_cb = on_avatar_status_cb

    def send_command(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.cmd_pub.publish(msg)

    def send_meta(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.meta_pub.publish(msg)

    def _status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.on_avatar_status_cb(data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON from /avatar/status: {msg.data}")


# --- Operator Console GUI ---

class OperatorConsoleApp:
    def __init__(self, root: tk.Tk, ros_node: ExperimentRosNode, log_dir: str = "./experiment_logs"):
        self.root = root
        self.ros = ros_node
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        self.root.title("HRI Experiment Operator Console")
        self.root.geometry("820x640")

        # Session state
        self.participant_id: Optional[str] = None
        self.log_path: Optional[str] = None
        self.study_block: str = "SETUP"  # SETUP, STUDY1, BREAK, STUDY2, DONE
        self.trial_state: str = "IDLE"   # IDLE, PROMPT_SHOWN, DELAY_ACTIVE, SPEAKING, RATING

        self.s1_trials: List[Study1Trial] = []
        self.s2_trials: List[Study2Trial] = []
        self.current_idx: int = 0
        self.current_timestamps: Dict[str, Optional[int]] = {}

        self._build_ui()
        self._bind_hotkeys()
        self._poll_ros()

    def _now_ms(self) -> int:
        return int(time.time() * 1000)

    def _poll_ros(self):
        rclpy.spin_once(self.ros, timeout_sec=0.01)
        self.root.after(20, self._poll_ros)

    def _bind_hotkeys(self):
        # Silent keyboard controls for the operator
        self.root.bind("<space>", lambda e: self.on_start_trial())
        self.root.bind("<Return>", lambda e: self.on_trigger_delay())
        self.root.bind("<n>", lambda e: self.on_next_trial())
        
    def _build_ui(self):
        # Top: Session Setup
        top = ttk.LabelFrame(self.root, text="Session Setup", padding=10)
        top.pack(fill="x", padx=10, pady=5)

        ttk.Label(top, text="Participant ID:").pack(side="left")
        self.pid_var = tk.StringVar()
        ttk.Entry(top, textvariable=self.pid_var, width=15).pack(side="left", padx=5)
        ttk.Button(top, text="Initialize Session", command=self.init_session).pack(side="left", padx=5)
        ttk.Button(top, text="Pause / Resume", command=self.toggle_pause).pack(side="right", padx=5)
        ttk.Button(top, text="Withdraw & Purge Data", command=self.withdraw_participant).pack(side="right", padx=5)

        # Middle: Status & Current Trial Info
        info = ttk.LabelFrame(self.root, text="Current Trial", padding=10)
        info.pack(fill="both", expand=True, padx=10, pady=5)

        self.status_lbl = ttk.Label(info, text="Block: SETUP | State: IDLE", font=("Arial", 12, "bold"))
        self.status_lbl.pack(anchor="w")

        self.meta_lbl = ttk.Label(info, text="Trial: - | Condition: -", font=("Arial", 11))
        self.meta_lbl.pack(anchor="w", pady=4)

        self.prompt_lbl = ttk.Label(info, text="", font=("Arial", 13), wraplength=760, foreground="#1a4e8a")
        self.prompt_lbl.pack(anchor="w", pady=10)

        # Controls
        ctrl = ttk.Frame(info)
        ctrl.pack(fill="x", pady=10)
        self.btn_start = ttk.Button(ctrl, text="1. Start Trial [Space]", command=self.on_start_trial, state="disabled")
        self.btn_start.pack(side="left", padx=5, ipadx=10, ipady=6)

        self.btn_delay = ttk.Button(ctrl, text="2. Trigger Delay [Enter]", command=self.on_trigger_delay, state="disabled")
        self.btn_delay.pack(side="left", padx=5, ipadx=10, ipady=6)

        self.btn_next = ttk.Button(ctrl, text="3. Complete & Next Trial [n]", command=self.on_next_trial, state="disabled")
        self.btn_next.pack(side="left", padx=5, ipadx=10, ipady=6)

        # Verbal Script Reference
        script = ttk.LabelFrame(self.root, text="Experimenter Verbal Script (Record on Paper Form)", padding=10)
        script.pack(fill="x", padx=10, pady=5)
        self.script_lbl = ttk.Label(
            script,
            text=(
                "Study 1 Per-Trial (1=Strongly Agree, 7=Strongly Disagree):\n"
                "  Q1: 'The wait time/delay for the response was too long.'\n"
                "  Q2: 'I found the avatar's screen presence to be engaging.'\n"
                "  Q3: 'The avatar's response felt very artificial.'"
            ),
            justify="left",
        )
        
        # Event logging at the
        self.script_lbl.pack(anchor="w")
        log_frame = ttk.LabelFrame(self.root, text="Live Event Log", padding=5)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.log_box = tk.Text(log_frame, height=8, state="disabled", font=("Courier", 9))
        self.log_box.pack(fill="both", expand=True)
 
    # --- Session Lifecycle ---

    def init_session(self):
        pid = self.pid_var.get().strip()
        if not pid:
            messagebox.showerror("Error", "Enter a randomized Participant ID first.")
            return
        self.participant_id = pid
        self.log_path = os.path.join(self.log_dir, f"{pid}_events.csv")

        # Replace with actual CSV loader for your pre-recorded (query, response, latency) triples
        # dummy_stimuli = {
        #     "short": [{"query_id": i, "query_text": f"Short query {i}?", "latency_ms": 1400} for i in range(1, 20)],
        #     "medium": [{"query_id": i, "query_text": f"Medium query {i}?", "latency_ms": 3500} for i in range(20, 40)],
        #     "long": [{"query_id": i, "query_text": f"Long query {i}?", "latency_ms": 6800} for i in range(40, 60)],
        # }
        # stimuli_by_cat = {"short": [], "medium": [], "long": []}
        # with open("/home/walleed/Avatar2/tools/stimuli.csv", "r") as f:
        #     for row in csv.DictReader(f):
        #         stimuli_by_cat[row["delay_category"]].append({
        #             "query_id": int(row["query_id"]),
        #             "query_text": row["query_text"],
        #             "response_text": row["response_text"],
        #             "latency_ms": int(row["latency_ms"]),
        #         })
        # categories = ["short", "medium", "long"]
        # for cat in categories:
        #     available = stimuli_by_cat[cat]
        #     if len(available) >= 15:
        #         pool = random.sample(available, 15)
        #     elif len(available) >= 5:
        #         # Use every query at least once, then top up to 15 without repeating inside a method
        #         pool = random.sample(available, len(available)) + random.sample(available, 15 - len(available))
        #     else:
        #         pool = random.choices(available, k=15)
        # self.s1_trials = generate_study1_schedule(stimuli_by_cat)
        # self.s2_trials = generate_study2_schedule()
        
        stimuli_by_cat = {"short": [], "medium": [], "long": []}
        with open("/home/walleed/Avatar2/tools/stimuli.csv", "r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                stimuli_by_cat[row["delay_category"]].append({
                    "query_id": int(row["query_id"]),
                    "query_text": row["query_text"],
                    "response_text": row["response_text"],
                    "latency_ms": int(row["latency_ms"]),
                })

        self.s1_trials = generate_study1_schedule(stimuli_by_cat)
        self.s2_trials = generate_study2_schedule()

        self._init_csv()
        self.study_block = "STUDY1"
        self.current_idx = 0
        self.trial_state = "IDLE"
        self._refresh_trial_display()

    def _init_csv(self):
        headers = [
            "participant_id", "study_block", "trial_num", "condition",
            "query_id", "recorded_latency_ms", "enforced_latency_ms",
            "trial_onset_ms", "delay_onset_ms", "filler_onset_ms",
            "response_onset_ms", "response_end_ms",
        ]
        with open(self.log_path, "w", newline="") as f:
            csv.writer(f).writerow(headers)

    def _refresh_trial_display(self):
        self.status_lbl.config(text=f"Block: {self.study_block} | State: {self.trial_state}")
        if self.study_block == "STUDY1":
            t = self.s1_trials[self.current_idx]
            tag = "PRACTICE" if t.is_practice else f"{t.trial_num}/45"
            self.meta_lbl.config(
                text=f"Trial: {tag} | Delay: {t.delay_category} ({t.enforced_latency_ms} ms) | Method: {t.presentation_method}"
            )
            self.prompt_lbl.config(text=f"Prompt: {t.query_text}")
            self.btn_start.config(state="normal")
            self.btn_delay.config(state="disabled")
            self.btn_next.config(state="disabled")

        elif self.study_block == "STUDY2":
            t = self.s2_trials[self.current_idx]
            self.meta_lbl.config(
                text=f"Trial: {t.trial_num}/24 | Pair {t.pair_id} | A: {t.modality_a}  vs  B: {t.modality_b} (5000 ms each)"
            )
            self.prompt_lbl.config(text="Press Start to run Presentation A followed by Presentation B.")
            self.script_lbl.config(text="Study 2 Per-Trial: Ask participant which presentation they preferred (A or B) and record on paper.")
            self.btn_start.config(state="normal")
            self.btn_delay.config(state="disabled")
            self.btn_next.config(state="disabled")

    # --- Trial Actions ---

    def on_start_trial(self):
        if self.trial_state != "IDLE":
            return
        now = self._now_ms()
        self.current_timestamps = {
            "trial_onset_ms": now,
            "delay_onset_ms": None,
            "filler_onset_ms": None,
            "response_onset_ms": None,
            "response_end_ms": None,
        }

        if self.study_block == "STUDY1":
            t = self.s1_trials[self.current_idx]
            self.trial_state = "PROMPT_SHOWN"
            self.ros.send_meta({"participant_id": self.participant_id, "block": "STUDY1", "trial": t.trial_num, "state": "PROMPT_SHOWN"})
            self.ros.send_command({"action": "SHOW_PROMPT", "query_id": t.query_id, "query_text": t.query_text})
            self.btn_start.config(state="disabled")
            self.btn_delay.config(state="normal")

        elif self.study_block == "STUDY2":
            t = self.s2_trials[self.current_idx]
            self.trial_state = "DELAY_ACTIVE"
            self.ros.send_meta({"participant_id": self.participant_id, "block": "STUDY2", "trial": t.trial_num, "state": "PAIR_PLAYING"})
            self.ros.send_command({
                "action": "RUN_STUDY2_PAIR",
                "modality_a": t.modality_a,
                "modality_b": t.modality_b,
                "delay_ms": t.delay_ms,
            })
            self.btn_start.config(state="disabled")
            self.btn_next.config(state="normal")

        self.status_lbl.config(text=f"Block: {self.study_block} | State: {self.trial_state}")

    def on_trigger_delay(self):
        if self.study_block != "STUDY1" or self.trial_state != "PROMPT_SHOWN":
            return
        t = self.s1_trials[self.current_idx]
        self.current_timestamps["delay_onset_ms"] = self._now_ms()
        self.trial_state = "DELAY_ACTIVE"

        self.ros.send_meta({"participant_id": self.participant_id, "block": "STUDY1", "trial": t.trial_num, "state": "DELAY_ACTIVE"})
        self.ros.send_command({
            "action": "START_DELAY",
            "query_id": t.query_id,
            "response_text": t.response_text,
            "delay_category": t.delay_category,
            "presentation_method": t.presentation_method,
            "enforced_latency_ms": t.enforced_latency_ms,
        })
        self.btn_delay.config(state="disabled")
        self.btn_next.config(state="normal")  # Enabled for fallback; also updated by Unity RESPONSE_END
        self.status_lbl.config(text=f"Block: {self.study_block} | State: {self.trial_state}")

    def handle_avatar_status(self, data: dict):
        event = data.get("event")
        ts = data.get("timestamp_ms", self._now_ms())
        if event == "FILLER_ONSET":
            self.current_timestamps["filler_onset_ms"] = ts
        elif event == "RESPONSE_ONSET":
            self.current_timestamps["response_onset_ms"] = ts
            self.trial_state = "SPEAKING"
        elif event == "RESPONSE_END":
            self.current_timestamps["response_end_ms"] = ts
            self.trial_state = "RATING"
            self.btn_next.config(state="normal")
        self.status_lbl.config(text=f"Block: {self.study_block} | State: {self.trial_state}")

    def on_next_trial(self):
        if self.study_block == "STUDY1":
            t = self.s1_trials[self.current_idx]
            if not t.is_practice:
                self._write_log_row("STUDY1", t.trial_num, f"{t.delay_category}_{t.presentation_method}",
                                    t.query_id, t.recorded_latency_ms, t.enforced_latency_ms)
            self.current_idx += 1
            if self.current_idx >= len(self.s1_trials):
                self.study_block = "STUDY2"
                self.current_idx = 0
                messagebox.showinfo("Study 1 Complete", "Administer 3 end-of-study items, then take the scheduled 10-minute break.")

        elif self.study_block == "STUDY2":
            t = self.s2_trials[self.current_idx]
            self._write_log_row("STUDY2", t.trial_num, f"{t.modality_a}_vs_{t.modality_b}", 0, t.delay_ms, t.delay_ms)
            self.current_idx += 1
            if self.current_idx >= len(self.s2_trials):
                self.study_block = "DONE"
                messagebox.showinfo("Session Complete", "Administer the 7 end-of-session items to conclude.")
                return

        self.trial_state = "IDLE"
        self._refresh_trial_display()

    def _write_log_row(self, block, trial_num, condition, query_id, rec_ms, enf_ms):
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([
                self.participant_id, block, trial_num, condition, query_id, rec_ms, enf_ms,
                self.current_timestamps.get("trial_onset_ms"),
                self.current_timestamps.get("delay_onset_ms"),
                self.current_timestamps.get("filler_onset_ms"),
                self.current_timestamps.get("response_onset_ms"),
                self.current_timestamps.get("response_end_ms"),
            ])

    def toggle_pause(self):
        self.ros.send_command({"action": "TOGGLE_PAUSE"})

    def withdraw_participant(self):
        if messagebox.askyesno("Confirm Withdrawal", "Delete all logged data for this participant immediately?"):
            if self.log_path and os.path.exists(self.log_path):
                os.remove(self.log_path)
            self.ros.send_command({"action": "ABORT_AND_PURGE", "participant_id": self.participant_id})
            messagebox.showinfo("Purged", "Participant data destroyed.")
            self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    app_ref = {}

    def status_cb(data):
        if "app" in app_ref:
            root.after(0, app_ref["app"].handle_avatar_status, data)

    ros_node = ExperimentRosNode(status_cb)
    app_ref["app"] = OperatorConsoleApp(root, ros_node)
    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()