import os
import sys
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

log_dir = "/home/ubuntu2204/Baro/Human-Simulator-ur5dex/logs/skrl/residual_dh_hand_catch/PPO_Residual_v8"
files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.startswith("events.out")]
files.sort(key=os.path.getmtime)
latest_file = files[-1]

print(f"Reading {latest_file}")
event_acc = EventAccumulator(latest_file)
event_acc.Reload()

tags = event_acc.Tags()['scalars']
for tag in tags:
    events = event_acc.Scalars(tag)
    if len(events) > 0:
        print(f"{tag}: last value {events[-1].value:.4f} at step {events[-1].step}")
